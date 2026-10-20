# IVR-Localization

Monte Carlo Localization (MCL) for a VEX V5 robot. Four distance sensors are compared against a
model of the field to correct LemLib odometry drift during a match.

Built with PROS (kernel 4.2.2) and LemLib 0.5.6.

## How it works

MCL runs alongside odometry (`angledOdom`: two angled tracking wheels plus the IMU) and corrects it. It
doesn't replace it, so when the sensors are unreliable the robot keeps running on odometry.

- `MCL::setTelemetry(&log)` (optional) gives MCL a `telemetry` log to write a frame to every cycle.
- `MCL::init(numParticles)` starts the MCL task. It's called from `initialize()`, but it's commented out
  there for now while PID is being tuned.
- `MCL::reset(startPose)` sets the odometry pose and re-scatters the particles around it. Call it once
  the robot's real starting pose is known.

The particles are created on the first cycle, in a Gaussian cloud around the odometry pose
(σ = 1.5" in position, 2° in heading). Poses are in the field frame described below (origin at field
centre). All tuning constants are at the top of `src/mcl.cpp`.

Each MCL cycle (every 100 ms, in its own task):

1. **Read sensors** – the four V5 Distance Sensors (front, right, back, left) are read and converted to
   inches. Each reading is one of:
   - a distance (20–2000 mm; readings past 200 mm also need a confidence of at least 30/63),
   - *nothing in range* (the sensor's 9999 code), or
   - *invalid* (sensor error or unplugged, closer than 20 mm, or low confidence); this carries no information.
2. **Update** – the odometry change since the last cycle is converted into the robot frame
   (forward / right) using the heading halfway through the move, which is more accurate on arcs. The
   change in heading is computed as `pose.theta - prevPose.theta`. Don't use `(pose - prevPose).theta`:
   LemLib's `Pose` subtraction keeps the left operand's theta. That move is then applied to every
   particle using the particle's own heading, plus noise that grows with the distance moved and the
   angle turned. Particles with the wrong heading drift away from the true path, and the sensors then
   rule them out.
3. **Weigh** – for each particle, `fieldRepresentation::simulateCast` predicts what each sensor should read
   and at what angle the beam hits the surface. Each sensor is scored with a beam model:
   - **hit** – Gaussian around the predicted distance. σ combines the sensor spec (±15 mm, or 5%), model
     error, and extra spread for oblique hits.
   - **short** – something between the sensor and the modelled surface, such as a robot or game element.
     This bounds how much an obstruction can hurt a particle.
   - **random** – uniform noise.
   - **nothing in range** counts against a particle only if it expects a close (< 48"), nearly head-on
     wall, because the sensor also returns nothing on glancing hits.

   The sensor scores are added (as log-likelihoods) to each particle's existing weight, so evidence builds
   up across cycles. The weights are then normalized to sum to 1. Particles off the field or inside a
   goal or loader get zero weight. If every particle is ruled out, the cloud is re-scattered around
   odometry at twice the starting spread. Weighing is skipped until the robot has moved 0.5" or turned
   1°, so a stationary robot doesn't keep reinforcing the same readings.
4. **Estimate** – the pose estimate is the weighted mean of the particles. Heading uses a circular mean
   of the heading offsets, so 179° and -179° average to 180°, not 0°. The cloud's spread (weighted RMS
   distance from the mean) measures how sure the filter is.
5. **Correct** – odometry is moved 30% of the way toward the estimate in position and 10% in heading,
   but only when the spread is under 3". A correction over 6" or 5° is treated as a glitch and skipped,
   unless it persists for 10 cycles in a row. The correction is excluded from the next cycle's odometry
   change, so particles aren't moved by it twice.
6. **Log** – if a telemetry log is set, the robot pose, sensor readings and every particle (pose, weight,
   simulated distances) are written as one CSV frame. This happens before resampling, so the weights
   are the ones just computed (see [Logging](#logging)).
7. **Resample** – only when the effective particle count drops below half the particle count.
   Resampling is low-variance (systematic): one random offset, then evenly spaced pointers through
   the cumulative weights, so particles are copied in proportion to their weight. Each copy gets a small
   jitter (0.1", 0.1°) so the copies don't stay identical, and all weights are reset to `1 / N`.
   Two running averages of how well the particles match the sensors are kept, one fast and one slow.
   When the fast one drops well below the slow one, the filter is probably lost, so up to 10% of
   particles are replaced with random ones within 12" of the odometry pose. They aren't spread across
   the whole field, because the field is symmetric.

Each particle stores its heading as an offset from the IMU (`heading = imu rotation + headingOffset`),
so the filter can also correct IMU drift.

`tools/mcl_sim/` runs this code on a PC against a simulated robot. In its run (two laps, 2% odometry
scale error, 0.1°/s IMU drift, a 4" push at 12 s), MCL gave a mean position error of 0.65" against
2.31" for odometry alone, and recovered from the push in about 3 s. The simulated sensors use the
same field model as MCL, so expect real runs to be noisier.

## Field model

The field is a set of axis-aligned boxes in inches. The origin is at field centre on top of the
foam tiles, +X points towards the blue alliance station and +Z is up. Headings are in degrees,
0° = +Y, clockwise positive, which matches LemLib and the IMU.

- **Walls** – one box spanning the inner faces of the perimeter, marked `interior` so a ray from
  inside the field hits where it exits.
- **Loaders and goals** – one box each.

Ray casting uses the slab method (ray vs. axis-aligned box), with:

- a per-sensor height filter built once in the constructor, since sensor heights don't change
- early exit when a box can't beat the closest hit found so far
- a guard for rays parallel to a box face
- the V5 distance sensor's 2000 mm (78.74") maximum range

The box coordinates come from the official field CAD. See [`tools/field/README.md`](tools/field/README.md)
for the scripts that extract them (`Resources/field-model/field_boxes.json`) and render the model.

## Telemetry viewer

`telemetryData/` is a small React + TypeScript (Vite) page for stepping through an MCL run. Upload the
CSV the robot writes to the SD card, then move through it one frame (one MCL step) at a time with
**Back** / **Forward**, or use **Play** / **Pause** / **Restart**. The frame index is shown with the time
in seconds, and playback runs at the log's `FramePeriodMs` (100 ms, the MCL loop period, for logs
without it). **Playback rate** scales that (2 = twice real time), and frames are skipped if drawing
can't keep up. Each frame draws, on a top-down 144" x 144" field:

- the robot at its odometry pose, sized by the **Robot width (x)** / **Robot length (y)** fields
  (inches, default 18 x 18),
- each distance sensor as a line from its mount point, as long as its reading, with the reading as a label
  (dashed to max range for *nothing in range*, a ring for *invalid*),
- every particle as a dot with a heading tick, colored from red (lowest weight in the frame) to green
  (highest). Clicking a particle highlights it in magenta and opens a table comparing each sensor's
  simulated distance from that particle with the measured reading, plus its weight.

Checkboxes toggle the robot, sensors and particles. **How to Use** (top right) explains how to add the
logger to other robot code.

### Logging

`telemetry` (`include/telemetry.h`, `src/telemetry.cpp`) writes the CSV. `initialize()` creates
`/usd/telemetry.csv` and passes it to `MCL::setTelemetry`, and MCL writes a frame at the end of every step.
The file starts with three metadata lines, followed by a header sized from the first two.
`FramePeriodMs` is the time between frames (`MCL::LOOP_MS`):

```
MaxParticles,300
NumDistanceSensors,4
FramePeriodMs,100
X,Y,Theta,NumParticles,DistanceSensorReading0,DistanceSensorOffset0,DistanceSensorDirection0,...,ParticlePosition0,ParticleWeight0,SimulatedDistance0_0,...
```

- `DistanceSensorReading` is in inches, with -1 for nothing in range and -2 for invalid.
- `DistanceSensorOffset` is `x;y` in the robot frame (+x right, +y forward).
- `DistanceSensorDirection` is in degrees, clockwise from the robot's front.
- `ParticlePosition` is `x;y;theta`.
- `ParticleWeight` is the particle's normalized weight from the last measurement update (written before resampling).
- `SimulatedDistance<i>_<j>` is particle *i*'s ray-cast for sensor *j*, with -1 for no hit.

Rows with fewer than `MaxParticles` particles leave the rest blank. Poses use the field-model
convention (inches from field centre, 0° = +Y, clockwise positive). The file is flushed every 10 frames.

To run it (Node.js required):

```
cd telemetryData
npm install
npm run dev
```

Then open the URL Vite prints. `npm run build` type-checks and builds to `telemetryData/dist/`.

## Layout

| File | Contents |
|---|---|
| `include/robot.h`, `src/robot.cpp` | Motors, IMU, tracking wheels, distance sensors, LemLib chassis |
| `include/fieldRepresentation.h`, `src/fieldRepresentation.cpp` | Field boxes, sensor layout, ray casting |
| `include/mcl.h`, `src/mcl.cpp` | MCL loop: read sensors, update, weigh, resample |
| `include/particle.h` | Particle pose (x, y, heading offset) and weight |
| `include/utilities.hpp` | `vector3`, `distanceSensor`, angle and unit conversions |
| `tools/field/` | Python scripts to build the field model from the CAD |
| `tools/mcl_sim/` | Runs MCL on a PC against a simulated robot and writes a telemetry CSV |
| `include/telemetry.h`, `src/telemetry.cpp` | CSV logger for poses, sensor readings and particles |
| `telemetryData/` | Web page to step through logged MCL frames on a drawing of the field |

## Building

With the PROS CLI installed:

```
pros make
pros upload
```

## Setup

- Set the distance sensor ports in `src/robot.cpp` (currently placeholders) and each sensor's mounting
  angle and offset from the robot's centre in `fieldRepresentation::distanceSensors()`.
- Mount the sensors level and more than about 3.5" above the tiles so the beam doesn't hit the floor.
- Call `chassis.calibrate()` and set the starting pose before `MCL::init`.

## Status

Work in progress.

- **Done** – field model and ray casting, the full MCL cycle (sensor reading, motion update,
  weighing, estimate, odometry correction, resampling and recovery), SD card logging, the telemetry
  viewer, and the PC simulator.
- **TODO** – set the real distance sensor ports, uncomment `MCL::init` in `initialize()` once PID
  tuning is done, and check MCL against real runs with the telemetry viewer.
