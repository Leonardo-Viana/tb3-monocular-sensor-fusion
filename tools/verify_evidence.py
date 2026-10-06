"""Verify migrated artifact bytes and saved counts without ROS or motion."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]

def main():
    manifest = json.loads((ROOT / 'data/provenance.json').read_text())
    errors = []
    for relative, expected in manifest['files_sha256'].items():
        path = ROOT / relative
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            errors.append('Missing or changed: ' + relative)
    archived = ROOT / 'experiments/2026-10-02-guarded'
    for path in (ROOT / 'runtime').iterdir():
        if path.is_file() and path.suffix in ('.py', '.yaml', '.rviz') and path.read_bytes() != (archived / path.name).read_bytes():
            errors.append('Runtime differs from initial guarded snapshot: ' + path.name)
    for label in ('2026-10-01-baseline', '2026-10-02-guarded'):
        stable = ROOT / 'experiments' / label / 'output/stable'
        validation = json.loads((stable / 'validation.json').read_text())
        count = len(json.loads((stable / 'stable_landmarks.json').read_text()))
        with (stable / 'monocular_stable.ply').open() as stream:
            header = []
            for line in stream:
                header.append(line.strip())
                if line.strip() == 'end_header':
                    break
        vertices = int(next(line for line in header if line.startswith('element vertex ')).split()[-1])
        if not count == vertices == validation['accepted_landmarks']:
            errors.append('Saved point counts disagree: ' + label)
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        raise SystemExit(1)
    print(f"PASS: {len(manifest['files_sha256'])} hashes, runtime provenance, and both mission point counts")

if __name__ == '__main__':
    main()
