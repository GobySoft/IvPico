# IvPico

This project aims to create lightweight applications for the IvP solver from [MOOS-IvP](https://oceanai.mit.edu/moos-ivp/).

Rather than using "bridges" or "gateways" to connect middlewares, this project creates first-class applications for IvP and the maritime behaviors in the middleware of choice (Goby, ROS, etc.).

One way to think of this project is as a generator for the equivalent of the MOOS-based pHelmIvP in your pub/sub middleware of choice.


## Repo Structure

- src: Source code
  - lib: Library source
    - interface: Interface messages (Protobuf)
  - bin: Binary source
    - goby: Goby middleware app
    - ros: ROS middleware node
- external: Submodules
  - openocean-messages: [openocean-messages](https://github.com/openocean-software/openocean-messages), for the `Navigation` and `ControlSetpoint` messages
  - moos-ivp: [MOOS-IvP](https://github.com/GobySoft/moos-ivp), for the IvP solver and behaviors

## Building

*This section was written by Claude.*

`init.sh` checks out the submodules and installs the Ubuntu packages needed to build them and IvPico. `build.sh` then builds openocean-messages from its submodule (into `build/external/openocean-messages`) and IvPico against it (into `build/`):

```
./init.sh
./build.sh
```

`build.sh` passes any arguments on to IvPico's CMake, and `BUILD_DIR` overrides the build directory.

MOOS-IvP isn't built by IvPico yet. To build its libraries and non-GUI apps (`build-moos.sh` calls skipping its GUI tools `--minrobot`, and MOOSGeodesy downloads PROJ while building):

```
cd external/moos-ivp
./build-moos.sh --minrobot
./build-ivp.sh --nogui
```

To update a submodule, check out the commit you want in it and commit the new pointer, e.g. `git -C external/openocean-messages pull origin main && git add external/openocean-messages`.
