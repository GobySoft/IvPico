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
- external: Submodules
  - moos-ivp: [MOOS-IvP](https://github.com/GobySoft/moos-ivp), for the IvP solver and behaviors

## Building

*This section was written by Claude.*

`init.sh` checks out the submodule and installs the Ubuntu packages needed to build it and IvPico. These include [openocean-messages](https://github.com/openocean-software/openocean-messages), for the `Navigation` and `ControlSetpoint` messages, adding [Open Ocean Software](https://github.com/openocean-software/openocean-build)'s continuous builds (apt.oceansoft.org) if apt doesn't already have it, and Goby3, for the Goby app, adding [packages.gobysoft.org](http://packages.gobysoft.org)'s continuous builds if apt doesn't already have it; `init.sh --no-goby` leaves Goby3 out, and `build.sh` then skips the Goby app. `build.sh` then builds, all in `build/`, and installs each into `build/usr`:

- MOOS-IvP's helm libraries (`IVP_BUILD_HELM_LIBS_ONLY`, as shared libraries, without MOOS or its apps), into `build/external/moos-ivp`, installed first so that IvPico can `find_package(IVP)`;
- IvPico, against them and openocean-messages.

```
./init.sh
./build.sh
ctest --test-dir build
```

`build.sh` passes any arguments on to IvPico's CMake, and `BUILD_DIR` overrides the build directory. Add `build/usr/bin` to your `PATH` to run the installed apps.

To update the submodule, check out the commit you want in it and commit the new pointer, e.g. `git -C external/moos-ivp pull origin ivp-without-moos && git add external/moos-ivp`.

## Goby app

*This section was written by Claude.*

`goby_helm_ivp` runs the helm in Goby3. It publishes and subscribes the `ivp.interface` messages on the `ivpico::helm::*` groups (`src/bin/goby/groups.h`), and can stand in for the MOOS helm behind `goby_frontseat_interface` (`frontseat { enable: true }`). `goby_helm_ivp -e` prints an example configuration.

`launch/goby/alpha` runs the alpha mission against Goby's basic frontseat simulator, with OpenCPN as a chart display: see its [README](launch/goby/alpha/README.md).

