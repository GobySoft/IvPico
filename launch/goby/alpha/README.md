# alpha

*This section was written by Claude.*

MOOS-IvP's alpha mission (a survey, then a return) with `goby_helm_ivp` driving Goby's basic frontseat simulator, at time warp 10.

```mermaid
flowchart LR
  helm["goby_helm_ivp"] -- "DesiredCourse" --> fs["goby_frontseat_interface"]
  fs <--> sim["goby_basic_frontseat_simulator"]
  fs -- "NodeStatus" --> helm
  fs -- "NodeStatus" --> ocpn["goby_opencpn_interface"]
  ocpn -- "AIS, TCP 54000" --> opencpn["OpenCPN"]
```

## Running

From the top of IvPico, after `./build.sh`:

```
export PATH=$PWD/build/usr/bin:$PATH
export GOBY_TOOL_LOAD_SHARED_LIBRARY=$PWD/build/usr/lib/libivpico_interface.so
cd launch/goby/alpha
goby_launch alpha.launch
./deploy.sh
```

`deploy.sh` publishes DEPLOY=true and lifts the manual override with `goby zeromq publish`, which can publish any other `Variable` or `Command` the same way.

## Viewing in OpenCPN

`goby_opencpn_interface` serves the vehicle's position as AIS on TCP port 54000 (`config/opencpn.pb.cfg`).

1. Install OpenCPN: `sudo add-apt-repository ppa:opencpn/opencpn && sudo apt install opencpn`.
2. In Options: Connections, add a Network connection: TCP, address 127.0.0.1, port 54000.
3. The vehicle appears as an AIS target near 43.825 N, 70.330 W. Charts are optional (Options: Charts: Chart Downloader).

See Goby's [visualization interfaces](https://github.com/GobySoft/goby3/blob/3.0/src/doc/markdown/doc507_goby_viz.md) for more.
