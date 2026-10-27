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
  - openocean-messages: [openocean-messages](https://github.com/openocean-software/openocean-messages), for the `Navigation` and `ControlSetpoint` messages
  - moos-ivp: [MOOS-IvP](https://github.com/GobySoft/moos-ivp), for the IvP solver and behaviors

## Building

*This section was written by Claude.*

`init.sh` checks out the submodules and installs the Ubuntu packages needed to build them and IvPico. These include Goby3 from [packages.gobysoft.org](http://packages.gobysoft.org), for the Goby app; without Goby3, `build.sh` skips the Goby app. `build.sh` then builds, all in `build/`:

- openocean-messages, into `build/external/openocean-messages`;
- MOOS-IvP's helm libraries (`IVP_BUILD_HELM_LIBS_ONLY`, as shared libraries, without MOOS or its apps), into `build/external/moos-ivp`, installed into `build/external/install` for `find_package(IVP)`;
- IvPico, against both.

```
./init.sh
./build.sh
ctest --test-dir build
```

`build.sh` passes any arguments on to IvPico's CMake, and `BUILD_DIR` overrides the build directory.

To update a submodule, check out the commit you want in it and commit the new pointer, e.g. `git -C external/openocean-messages pull origin main && git add external/openocean-messages`.

## Goby app

*This section was written by Claude.*

`goby_helm_ivp` runs the helm in Goby3. It publishes and subscribes the `ivp.interface` messages on the `ivpico::helm::*` groups (`src/bin/goby/groups.h`), and can stand in for the MOOS helm behind `goby_frontseat_interface` (`frontseat { enable: true }`). `goby_helm_ivp -e` prints an example configuration.

To run the alpha mission against Goby's basic frontseat simulator:

```
export PATH=$PWD/build/src/bin/goby/helm_ivp:$PATH
export GOBY_TOOL_LOAD_SHARED_LIBRARY=$PWD/build/src/lib/libivpico_interface.so
cd launch/goby/alpha
goby_launch alpha.launch
./deploy.sh
```

`deploy.sh` publishes DEPLOY=true and lifts the manual override with `goby zeromq publish`, which can publish any other `Variable` or `Command` the same way.

