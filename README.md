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

`init.sh` checks out the submodules and installs the Ubuntu packages needed to build them and IvPico. Then build openocean-messages and point IvPico at it:

```
./init.sh
external/openocean-messages/build.sh
cmake -S . -B build -Dopenocean_messages_DIR=$PWD/external/openocean-messages/build
cmake --build build
```

MOOS-IvP isn't built by IvPico yet. To build its libraries and non-GUI apps (MOOSGeodesy downloads PROJ while building):

```
cd external/moos-ivp
./build-moos.sh
./build-ivp.sh --nogui
```

To update a submodule, check out the commit you want in it and commit the new pointer, e.g. `git -C external/openocean-messages pull origin main && git add external/openocean-messages`.
