"""Prepare a fresh working directory. Never launch ROS or robot motion."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', help='New local run name: letters, digits, underscores or hyphens')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', args.name):
        parser.error('Use a simple run name, not a path')
    destination = ROOT / 'runs' / args.name
    destination.mkdir(parents=True, exist_ok=False)
    for source in (ROOT / 'runtime').iterdir():
        if source.is_file():
            shutil.copyfile(source, destination / source.name)
    scene = destination / 'scene'
    scene.mkdir()
    pilot = ROOT / 'experiments/2026-09-30-texture-pilot'
    shutil.copyfile(pilot / 'build_worlds.py', scene / 'build_worlds.py')
    shutil.copytree(pilot / 'source', scene / 'source')
    hashes = {str(p.relative_to(destination)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in destination.rglob('*') if p.is_file()}
    (destination / 'PREPARATION.json').write_text(json.dumps({
        'status': 'prepared_only_no_processes_started',
        'files_sha256': hashes,
        'notes': 'Regenerate scene paths and review docs/reproduction.md before launching.'
    }, indent=2) + '\n')
    print(destination)

if __name__ == '__main__':
    main()
