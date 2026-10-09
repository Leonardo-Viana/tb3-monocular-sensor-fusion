# Replication evidence — 7 October 2026

Three completed runs, analyzed on 8 October. The archive contains 188 manifest-listed files plus MANIFEST.json, including three PLY maps, frozen launch sources, metrics, selected logs, derived resource traces, test residuals and exploratory revisit correspondences.

Archive SHA-256: `5ec6a3c494305eae9ca7e15ef5cbb438d0e0ad401ef62a5d7acb79bb8a81f2d5`. Verify and regenerate the English report with `python3 tools/report_campaign.py data/publication-evidence-20261007.zip .`. Plot generation is documented in that report.

Extract into a fresh directory to inspect the maps at `repeat_01/output/stable/monocular_stable.ply`, and similarly for repeats 02 and 03. Do not extract over historical experiment directories.

Full raw MCAP, JPEG images, keyframes, tracks and original audit/resource traces remain on the Dell under `/home/lpviana/tb3_publication_campaign_20261007`. Their locations and selected source hashes are recorded in MANIFEST.json; all MCAP hashes and per-topic message counts are in analysis/ANALYSIS.json. This compact archive is not a full replay dataset. The original records have not been removed.

See [the English analysis](../paper/replication-report-20261007.md) for limitations, software exceptions and publication readiness. Repository licensing applies to this archive and extracted files.
