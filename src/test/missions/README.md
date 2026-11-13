# Mission replay tests

*This section was written by Claude.*

These tests check that the IvPico helm (`src/lib/helm`) does what pHelmIvP does, using MOOS-IvP's own missions.

```mermaid
flowchart LR
  record["record.sh<br/>(all of MOOS-IvP, headless)"] -- "alog, .moos, .bhv" --> fixtures["fixtures/&lt;mission&gt;"]
  fixtures --> replay["replay<br/>(ctest: replay_&lt;mission&gt;)"]
  replay -- "same inputs, same iteration times" --> helm["ivpico::helm::Helm"]
  replay -- "compare" --> report["pass / fail per stream"]
```

- `missions.txt` lists the missions and the variables poked to start them.
- `record.sh [mission...]` builds all of MOOS-IvP from GobySoft's `ivp-without-moos` branch (the source of the `libivp-helm-dev` package) into `build/missions`, runs each mission under pHelmIvP with time warp, and keeps each vehicle's helm inputs and outputs in `fixtures/<mission>`. Missions with several vehicles run from their own `launch.sh --just_make`, with a fixture per vehicle, `fixtures/<mission>.<vehicle>`. The "Record missions" workflow runs it on GitHub and uploads the fixtures as an artifact.
- `replay <fixture>` feeds the recorded inputs to the IvPico helm at pHelmIvP's iteration times and compares what each helm posted. `replay.cpp` describes what's compared and the tolerances.
- `replay <fixture> --alerts` (ctest: `replay_<fixture>_alerts`, for vehicles that ran pContactMgrV20) leaves out pContactMgrV20's recorded posts and has the helm raise contact alerts itself, from the `.moos` file's pContactMgrV20 block.

Re-record the fixtures whenever `libivp-helm-dev` moves to a new MOOS-IvP.

## Closed-loop runs under Goby

*This section was written by Claude.*

`goby test` runs `goby/<mission>/test.yml` (ctest: `closed_loop_<mission>`), which flies a mission's recorded `.bhv` with `goby_helm_ivp` against Goby's basic frontseat simulator, with time warp 10:

```mermaid
flowchart LR
  test["goby test"] -- "pokes (Variable)" --> helm["goby_helm_ivp"]
  helm -- "Posting, Status" --> test
  helm -- "DesiredCourse" --> fs["goby_frontseat_interface<br/>+ basic simulator"]
  fs -- "NodeStatus" --> helm
```

- `goby/closed_loop.yml` starts the fixtures (`gobyd`, the simulator, the frontseat interface and the helm) and waits for them; each `test.yml` includes it.
- `goby/<mission>/test.yml` sends the pokes and expects postings by a deadline. Its times are real seconds, a tenth of mission time.
- `goby/<mission>/helm.pb.cfg` is the helm configuration (the `.moos` file's pHelmIvP block, and its pContactMgrV20 block as `contact_manager`).
- In `m2_berta`, `ivpico_ghost` (`src/test/goby/ghost`) replays the other vehicle's recorded NODE_REPORTs from the helm's first DRIVE. The ghost doesn't react to ownship. It publishes its range to ownship, which the test keeps above a minimum.
- `goby test visualize <test.yml> -D MISSION=<mission> ...` draws the plan; a failed run's report names each fixture's log.
