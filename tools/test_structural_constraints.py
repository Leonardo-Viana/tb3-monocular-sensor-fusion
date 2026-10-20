"""Analytic failure-case checks, not TurtleBot benchmark evidence."""
import unittest
import numpy as np
from structural_constraints import (vertical_wall, camera_ray, intersect_wall,
                                    select_supported_candidate, same_mask_support)


class StructuralGeometryTests(unittest.TestCase):
    def setUp(self):
        self.wall = vertical_wall([3., -2., .2], [3., 2., .2])

    def test_known_height(self):
        hit, _ = intersect_wall([0, 0, .3], [1, .4, .2], self.wall)
        np.testing.assert_allclose(hit['xyz'], [3, 1.2, .9])
        self.assertIn('requires_association', hit['status'])

    def test_ray_scaling_does_not_change_point_or_range(self):
        a, _ = intersect_wall([0, 0, .3], [1, .4, .2], self.wall)
        b, _ = intersect_wall([0, 0, .3], [10, 4, 2], self.wall)
        np.testing.assert_allclose(a['xyz'], b['xyz'])
        self.assertAlmostEqual(a['range_m'], b['range_m'])

    def test_rigid_frame_change_invariance(self):
        # Rotate the entire problem so gravity is no longer world z.
        R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.]])
        t = np.array([7., 5., -2.])
        w = vertical_wall(R@self.wall.p0+t, R@self.wall.p1+t, R@self.wall.up)
        h, _ = intersect_wall(R@np.array([0, 0, .3])+t, R@np.array([1, .4, .2]), w)
        np.testing.assert_allclose(h['xyz'], R@np.array([3, 1.2, .9])+t)

    def test_tilted_scan_endpoints(self):
        w = vertical_wall([3, -2, -.1], [3, 2, .7])
        h, _ = intersect_wall([0, 0, .3], [1, .4, .2], w)
        np.testing.assert_allclose(h['xyz'], [3, 1.2, .9])

    def test_parallel_and_grazing_rays_rejected(self):
        for ray in ([0, 1, 0], [.001, 1, 0]):
            self.assertEqual(intersect_wall([0, 0, .3], ray, self.wall)[1],
                             'grazing_or_parallel')

    def test_behind_camera_rejected(self):
        self.assertEqual(intersect_wall([0, 0, .3], [-1, 0, 0], self.wall)[1],
                         'behind_camera')

    def test_finite_support_not_infinite_wall(self):
        self.assertEqual(intersect_wall([0, 0, .3], [1, 1, 0], self.wall)[1],
                         'outside_segment_support')

    def test_range_limit_is_euclidean(self):
        self.assertEqual(intersect_wall([0, 0, .3], [1, .4, .2], self.wall,
                                       max_range=3.1)[1], 'range')

    def test_optical_frame_convention(self):
        T = np.eye(4)
        T[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        T[:3, 3] = [0, 0, .3]
        K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1.]])
        c, r = camera_ray([120, 140], K, T)
        h, _ = intersect_wall(c, r, self.wall)
        np.testing.assert_allclose(h['xyz'], [3, 1.2, .9])

    def test_invalid_calibration_and_nan_rejected(self):
        T = np.eye(4); T[0, 0] = -1
        with self.assertRaises(ValueError): camera_ray([1, 2], np.eye(3), T)
        with self.assertRaises(ValueError): intersect_wall([np.nan, 0, 0], [1, 0, 0], self.wall)
        with self.assertRaises(ValueError): vertical_wall([0, 0, 0], [0, 0, 1])

    def test_foreground_visual_point_rejects_background_plane(self):
        h, _ = intersect_wall([0, 0, .3], [1, 0, 0], self.wall)
        self.assertEqual(select_supported_candidate([h], [1, 0, .3])[1],
                         'no_visual_support')

    def test_competing_structures_are_ambiguous(self):
        h, _ = intersect_wall([0, 0, .3], [1, 0, 0], self.wall)
        self.assertEqual(select_supported_candidate([h, h], [3, 0, .3])[1],
                         'ambiguous_structure')

    def test_unique_structure_is_only_a_candidate(self):
        h, _ = intersect_wall([0, 0, .3], [1, 0, 0], self.wall)
        self.assertEqual(select_supported_candidate([None, h], [3, 0, .3])[1],
                         'supported_candidate_requires_visibility')

    def test_mask_boundary_occlusion_and_duplicate_support(self):
        mask = np.zeros((30, 30), bool); mask[5:25, 5:25] = True
        pts = [[8, 10], [10, 10], [12, 10]]
        self.assertTrue(same_mask_support(mask, [15, 15], pts))
        self.assertFalse(same_mask_support(mask, [5, 15], pts))
        self.assertFalse(same_mask_support(mask, [-.1, 15], pts))
        self.assertFalse(same_mask_support(mask, [15, 15], [[1, 1], [2, 2], [3, 3]]))
        self.assertFalse(same_mask_support(mask, [15, 15], [[10, 10]]*5))


if __name__ == '__main__':
    unittest.main(verbosity=2)
