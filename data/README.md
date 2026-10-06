# Data inventory and provenance

`provenance.json` records the SHA-256 and size of each input archive, the repository destination, and hashes of every preserved archive member plus the initial runtime copy. Each experiment also retains its original protocol, source hashes and data checksums. Original full-dataset manifests can mention files intentionally omitted from the portable archive.

The repository contains final sparse PLY/JSON maps, occupancy snapshots, validation and revisit metrics, selected images, command/resource audits, raw-bag metadata, and English reports. It retains the original baseline failure/resume history.

Full MCAP payloads and most raw RGB frames are not tracked. The guarded run alone has 4,077,517,114 bytes of MCAP recordings on the Dell. Its `output/FINAL_REPORT.json` records message counts and per-file hashes. Matching counts and decoded endpoint RGB images establish checks on the stored recording, not zero acquisition loss.

No external public dataset URL or DOI exists yet. Raw-data publication, redistribution scope and retention should be defined with the paper. Small exported maps and result tables stay in Git; new raw recordings remain under ignored run directories or separately managed data storage.

Archived logs include historical local paths and process IDs needed to interpret the experiment. They must not be used to terminate or control current processes. Do not reinterpret old completion timestamps as current robot status.
