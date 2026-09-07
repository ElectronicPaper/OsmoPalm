# Two projects, shared learning

[OsmoPalm](https://github.com/ElectronicPaper/OsmoPalm) owns Core2 firmware, on-device controls, IMU processing, haptics, standalone pairing, and embedded motion.
[OsmoDesk](https://github.com/ElectronicPaper/OsmoDesk) owns browser/mobile interfaces, the Python camera host, live view, shot planning, and optional USB integration.

Check them out as siblings. Open the local OsmoControllers.code-workspace to navigate both.
With a coding agent, open their parent directory for a task that explicitly needs changes in both.
Repository access still comes from your GitHub account; this does not share credentials or grant either program access to the other.

## How to carry experience across

1. Consult the sibling's relevant implementation, tests, docs/LESSONS.md and recent commits.
2. Treat a finding as a candidate, not a universal fix: embedded timing, radio constraints and host scheduling differ.
3. When porting a proven fix, identify the source repo/commit, the failure it prevents, the adaptation, and target-specific tests in the receiving change.
4. For protocol changes, update the versioned fixture in both repositories deliberately and run both local suites plus the paired check.
5. Commit each project independently. Link the companion commit or issue. Do not copy whole firmware/driver trees to avoid a contract change.

## Compatibility, not coupling

contracts/camera-control-v1.json is a checked-in baseline, not a runtime configuration file.
Both suites assert their implementation against it; neither fetches a sibling during normal tests.
The contract contains public protocol constants and symbolic action names, never camera passwords.
OsmoDesk's integration_tests/test_paired_projects.py checks the actual sibling sources and
requires OSMOPALM_ROOT explicitly; a missing checkout fails rather than silently skipping.
From OsmoDesk, with its Python environment and OSMOPALM_ROOT set:

```sh
python -B -m unittest discover -s integration_tests -t .
```

The fixture is mirrored intentionally, not auto-synced. Incompatible changes require a new
contract revision and an explicit migration; do not quietly alter v1 semantics.
The applications remain usable independently. OsmoDesk can optionally use an OsmoPalm
over the existing USB protocol, but standalone OsmoPalm never needs the desktop host.

## Public release boundary

Both repositories start private with a clean initial snapshot. Public users will see main
when visibility is explicitly changed later. Do not fabricate historical commits or
remove upstream attributions. Before that change, review all commits for secrets and
third-party provenance, select a project license, review names/trademarks, and confirm
hardware/browser acceptance. A clean snapshot alone is not a security or product certification.
