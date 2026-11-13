# IvPico

This project aims to create lightweight applications for the IvP solver from [MOOS-IvP](https://oceanai.mit.edu/moos-ivp/).

Rather than using "bridges" or "gateways" to connect middlewares, this project creates first-class applications for IvP and the maritime behaviors in the middleware of choice (Goby, ROS, etc.).

One way to think of this project is as a generator for the equivalent of the MOOS-based pHelmIvP in your pub/sub middleware of choice.


## Repo Structure

- src: Source code
  - lib: Library source
    - interface: Interface messages (Protobuf)
    - helm: The helm, pHelmIvP without MOOS (`ivpico::helm::Helm`)
  - bin: Binary source
    - goby: Goby middleware app (`goby_helm_ivp`)
    - ros: ROS middleware node
  - test: Tests, including replays of MOOS-IvP missions (`test/missions`)
- launch: Example missions

## Building

*This section was written by Claude.*

`init.sh` installs the Ubuntu packages needed to build IvPico:

- [openocean-messages](https://github.com/openocean-software/openocean-messages), for the `Navigation` and `ControlSetpoint` messages, and MOOS-IvP's helm libraries built without MOOS ([GobySoft's `ivp-without-moos`](https://github.com/GobySoft/moos-ivp/tree/ivp-without-moos)), for the IvP solver and behaviors.
- Goby3, for the Goby app. `init.sh --no-goby` leaves Goby3 out, and `build.sh` then skips the Goby app.

All of these come from [Open Ocean Software](https://github.com/openocean-software/openocean-build)'s continuous builds, with its releases for what they depend on (DCCL, MOOS, ...), at apt.oceansoft.org. `init.sh` adds both if apt can't install the packages already.

`build.sh` then builds IvPico in `build/` and installs it into `build/usr`.

```
./init.sh
./build.sh
ctest --test-dir build
```

`build.sh` passes any arguments on to IvPico's CMake, and `BUILD_DIR` overrides the build directory. Add `build/usr/bin` to your `PATH` to run the installed apps.

## Helm library

*Outline, to be written:*

- `ivpico::helm::Helm`: construct from a `HelmConfig`, `handle()` the inputs, `iterate()` at the helm's rate.
- Each iteration's `Output` (solution, status, postings, behavior events, visualizations, offboard messages, life events, objective functions, configuration) and its pHelmIvP equivalent.
- Field levels (CORE, ADVANCED, DEVELOPER) and what each adds to `Status` and `HelmConfig`.
- Differences from pHelmIvP: the MOOSDB echo, rejournal, standby helm.

## Goby app

*This section was written by Claude.*

`goby_helm_ivp` runs the helm in Goby3. It publishes and subscribes the `ivp.interface` messages on the `ivpico::helm::*` groups (`src/bin/goby/groups.h`), and can stand in for the MOOS helm behind `goby_frontseat_interface` (`frontseat { enable: true }`). `goby_helm_ivp -e` prints an example configuration.

Contacts come in as `Navigation` with `self: false`, or as `NODE_REPORT` variables. The helm raises pContactMgrV20's alerts itself (configured in `helm.contact_manager`), so contact behaviors such as collision avoidance spawn without a separate contact manager.

`launch/goby/alpha` runs the alpha mission against Goby's basic frontseat simulator, shown on Liaison's map: see its [README](launch/goby/alpha/README.md).


## License

The libraries (`src/lib`) are LGPL 2.1 or later, and the binaries and tests (`src/bin`, `src/test`) are GPL 2 or later. The helm's ports of pHelmIvP and pContactMgrV20 (`src/lib/helm/helm.cpp`, `contact_manager.{h,cpp}` and `contacts/`) are GPL 3 or later, as is MOOS-IvP, whose helm libraries IvPico links. See [COPYING](COPYING).
