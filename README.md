# The A-Team, Avishkar 2020 (Guardian)

Autonomous drone flight scripts written for the Guardian event at Avishkar 2020. Everything here runs in simulation: a single quadcopter flies geometric patterns in Round 1, and two- and three-drone swarms fly coordinated formations in Round 2.

The flight logic is plain Python on top of DroneKit and pymavlink, talking MAVLink to ArduPilot SITL. Round 1 problem 1.3 was flown in AirSim for the visuals; the rest runs against dronekit-sitl or the ArduCopter SITL build.

## How the patterns are built

Every mission works the same way. The shape is first solved in flat Cartesian coordinates with the drone's home position as the origin, then each (x, y) point is converted to latitude and longitude:

```
new_lon = home_lon + x * 180 / (pi * EARTH_RADIUS * cos(home_lat * pi / 180))
new_lat = home_lat + y * 180 / (pi * EARTH_RADIUS)
```

For vertical patterns the latitude is held fixed and `y` is added to the takeoff altitude instead, so the circle or figure-8 is traced in a vertical plane. The resulting points become `MAV_CMD_NAV_WAYPOINT` commands uploaded to the vehicle, and the script flips the copter into AUTO and watches `vehicle.commands.next` until the last waypoint is reached.

Circles and figure-8s come from solving `x² + y² = r²` in pieces: semicircles and quarter-circles stitched end to end. The spiral is six semicircles of growing radius with alternating centres. The hexagon is just its six vertices.

## What is in each round

Round 1, single drone (`Round 1/Codes/`):

- `Problem_1.1_1.2.py` flies a hexagon with 35 m sides, a horizontal figure-8, and a spiral built from six semicircles, all as uploaded waypoint missions at 15 m.
- `Problem_1.3.py` flies a vertical circle and a vertical figure-8, then does a flip. The flip is done by overriding RC channel 3 to climb past 30 m and switching the flight mode to FLIP.

Round 2, swarm (`Round 2/Codes/`):

- `Problem_2.1_2.2.py` connects to two vehicles at once. Vehicle 1 flies the vertical figure-8 and vertical circle at 45 m as an AUTO waypoint mission while vehicle 2 flies the horizontal versions at 15 m, driven point by point with `simple_goto()`.
- `Problem_2.3.py` connects to three vehicles and holds four formations: triangle, line, L, and vertical line. Vehicle 1 is the leader; each drone is sent to its own slot in the formation with `simple_goto()`, computed from vehicle 1's position and a 30 m side length.

Each round also ships a Word and PDF write-up of the code (`Documentaton of codes`) and the exact SITL and MAVProxy commands used, including the home coordinates for each drone in the swarm runs.

## Running it

You need ArduPilot SITL (or dronekit-sitl) and a ground station such as Mission Planner or MAVProxy. `Round 1/Steps for setting up Ardupilot-SITL and AirSim.docx` walks through the Cygwin and AirSim setup on Windows.

Install the Python side:

```
pip install dronekit dronekit-sitl pymavlink numpy
```

For Round 1, start a simulated copter and connect the script to it:

```
dronekit-sitl copter --home=25.495007,81.864489,0,90
python mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551

python "Round 1/Codes/Problem_1.1_1.2.py" --connect udp:127.0.0.1:14550
```

Running with no `--connect` argument starts a default SITL instance automatically.

For AirSim, launch the AirSim binary first, then:

```
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f airsim-copter
```

The Round 2 scripts expect their vehicles at fixed UDP endpoints (`127.0.0.1:14550`, `127.1.1.1:14551`, `127.2.2.1:14552`), so start one SITL instance per drone with matching `-I` indices. `Round 2/Swarm Commands.txt` has the full command list.

A note on flight modes: the scripts cannot always force a mode change through, so they loop and print a prompt asking you to set GUIDED, AUTO, or RTL by hand in the GCS. That is expected, not a hang.

## Video

Round 1, problems 1.1 and 1.2 plus part of 1.3: https://youtu.be/xl8mHPBc3Bw

Round 1, problem 1.3: https://youtu.be/Xmt8UnFxhJM

Round 2: https://youtu.be/2WiYfri0MZE

## Team

The A-Team: Ankur Pratap Singh (20191020)

## License

MIT. See [LICENSE](LICENSE).
