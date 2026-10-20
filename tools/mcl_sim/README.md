# MCL simulator

Runs the real `src/mcl.cpp`, `src/fieldRepresentation.cpp` and `src/telemetry.cpp` on a PC against a
simulated robot, and writes the same telemetry CSV the robot writes to the SD card. Open the CSV in
the telemetry viewer (`telemetryData/`) to watch the run.

```
cmake -S tools/mcl_sim -B tools/mcl_sim/build
cmake --build tools/mcl_sim/build --config Release
tools/mcl_sim/build/Release/mcl_sim.exe tools/mcl_sim/out/sim.csv
```

(On Windows this needs Visual Studio with the C++ workload; on Linux/macOS the exe is `build/mcl_sim`.)
To build against a modified copy of MCL without touching `src/`, configure with `-DMCL_SOURCE=path/to/mcl.cpp`.

## What it simulates

- `stubs/` replaces PROS, LemLib and `angledOdom` with small stand-ins. MCL's task runs in a thread
  that steps in lockstep with the simulation, once per 100 ms. `lemlib::Pose::operator-` keeps the
  left operand's theta, the same as LemLib.
- The robot starts at (-34, -34) facing +X and drives two laps of a square at ±34", turning in place
  at each corner. It steers by its odometry, the way a LemLib motion would.
- Odometry reads 2% long, with a little noise and sideways slip. The IMU drifts 0.1°/s.
- At 12 s the robot is pushed 4" in +Y, and odometry doesn't see it.
- Each distance sensor ray-casts from the true pose against `fieldRepresentation` and adds noise at the
  V5 spec (±15 mm under 200 mm, 5% beyond). Glancing hits and anything past 2000 mm read 9999.
  3% of readings are short (something in the way), and 3% have low confidence.
- The console prints position and heading error over time, for odometry corrected by MCL and for
  odometry alone.

The simulated sensors use the same field model as MCL, so map error isn't simulated. Real runs will be
noisier.
