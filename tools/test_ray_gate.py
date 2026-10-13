"""Analytic geometry checks independent of the two observed tracking failures."""
import json
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'runtime'))
from ray_gate import correspondence_mask, ray_segment_distances


class RayGateTests(unittest.TestCase):
    def setUp(self):
        self.K = np.array([[600., 0, 480], [0, 600., 270], [0, 0, 1.]])
        self.A = np.eye(4)

    def project(self, xyz, pose):
        local = (xyz-pose[:3, 3]) @ pose[:3, :3]
        q = local @ self.K.T
        return q[:, :2]/q[:, 2:], local[:, 2]

    def test_identical_pose_accepts_same_ray_rejects_large_shift(self):
        old = np.array([[480., 270.], [123., 99.]])
        self.assertTrue(correspondence_mask(old, old, self.A, self.A, self.K).all())
        self.assertFalse(correspondence_mask(old, old+[20, 0], self.A, self.A, self.K).any())

    def test_rotating_camera_predicts_motion_and_rejects_zero_flow(self):
        B = self.A.copy(); B[:3, :3] = Rotation.from_euler('y', .14).as_matrix()
        xyz = np.array([[0., 0., 2.], [.2, .1, 3.]])
        a, _ = self.project(xyz, self.A); b, _ = self.project(xyz, B)
        self.assertTrue(correspondence_mask(a, b, self.A, B, self.K).all())
        self.assertFalse(correspondence_mask(a, a, self.A, B, self.K).any())

    def test_depth_clipping_in_second_camera(self):
        B = self.A.copy(); B[2, 3] = .3
        xyz = np.array([[.03, .02, .46], [.1, -.1, 5.9]])
        a, _ = self.project(xyz, self.A); b, _ = self.project(xyz, B)
        self.assertTrue(correspondence_mask(a, b, self.A, B, self.K).all())

    def test_transverse_translation_rejects_opposite_parallax(self):
        B = self.A.copy(); B[0, 3] = .1
        a = np.array([[480., 270.]])
        self.assertTrue(correspondence_mask(a, [[450., 270.]], self.A, B, self.K)[0])
        self.assertFalse(correspondence_mask(a, [[510., 270.]], self.A, B, self.K)[0])

    def test_known_valid_random_scene_and_world_frame_invariance(self):
        rng = np.random.default_rng(981)
        xyz = np.c_[rng.uniform(-.3, .3, (1000, 2)), rng.uniform(.4, 5.5, 1000)]
        B = self.A.copy(); B[:3, :3] = Rotation.from_euler('xyz', [.03, -.10, .02]).as_matrix(); B[:3, 3] = [.04, -.02, .03]
        a, _ = self.project(xyz, self.A); b, z = self.project(xyz, B)
        valid = (z >= .15) & (z <= 6)
        d = ray_segment_distances(a[valid], b[valid], self.A, B, self.K)
        self.assertLess(d.max(), 1e-8)
        W = self.A.copy(); W[:3, :3] = Rotation.from_euler('xyz', [.2, -.3, .7]).as_matrix(); W[:3, 3] = [5., 8., 1.]
        np.testing.assert_allclose(d, ray_segment_distances(a[valid], b[valid], W@self.A, W@B, self.K), atol=1e-8)

    def test_nonfinite_and_singular_inputs_fail_closed(self):
        self.assertFalse(correspondence_mask([[np.nan, 1]], [[1, 1]], self.A, self.A, self.K)[0])
        self.assertFalse(correspondence_mask([[1, 1]], [[1, 1]], self.A, self.A, np.zeros((3, 3)))[0])
        self.assertFalse(correspondence_mask([[1, 1]], [[np.inf, 1]], self.A, self.A, self.K)[0])

    def test_camera_facing_away_has_no_positive_depth_segment(self):
        B = self.A.copy(); B[:3, :3] = Rotation.from_euler('y', np.pi).as_matrix()
        self.assertFalse(correspondence_mask([[480, 270]], [[480, 270]], self.A, B, self.K)[0])


class RecordedFailureTests(unittest.TestCase):
    def test_preserved_bad_transitions(self):
        path=Path(__file__).resolve().parents[1]/'data/ray-gate-fixtures-20261013.json'
        if not path.exists():
            self.skipTest('Compact recorded fixture is not installed in this copy')
        for f in json.loads(path.read_text()):
            with self.subTest(track_id=f['track_id']):
                accepted=correspondence_mask([f['previous_uv']],[f['current_uv']],
                    f['previous_T'],f['current_T'],f['K'])
                self.assertFalse(accepted[0])


if __name__ == '__main__':
    unittest.main(verbosity=2)
