"""Restore byte-preserved experiment trees from the distributed ZIP archives."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]

def main():
    manifest = json.loads((ROOT / 'data/provenance.json').read_text())
    pending = []
    for item in manifest['archives']:
        archive = ROOT / item['archive']
        if hashlib.sha256(archive.read_bytes()).hexdigest() != item['sha256']:
            raise SystemExit('Archive checksum mismatch: ' + archive.name)
        with zipfile.ZipFile(archive) as z:
            for member in z.infolist():
                if member.is_dir():
                    continue
                parts = Path(member.filename).parts
                if len(parts) < 2 or '..' in parts or Path(member.filename).is_absolute():
                    raise SystemExit('Unsafe archive member')
                rel = Path(item['destination']) / Path(*parts[1:])
                target = ROOT / rel
                data = z.read(member)
                expected = manifest['files_sha256'][str(rel)]
                if hashlib.sha256(data).hexdigest() != expected:
                    raise SystemExit('Member checksum mismatch: ' + str(rel))
                if target.exists() and target.read_bytes() != data:
                    raise SystemExit('Refusing to overwrite changed evidence: ' + str(rel))
                pending.append((target, data))
    # Validate all archives and existing files before making any writes.
    for target, data in pending:
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            target.write_bytes(data)
    print(f'Restored or verified {len(pending)} historical files; no ROS or robot process started')

if __name__ == '__main__':
    main()
