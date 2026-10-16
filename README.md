# IVR-Localization

Monte Carlo Localization (MCL) for a VEX V5 robot. Four distance sensors are compared against a
model of the field to correct LemLib odometry drift during a match.

Built with PROS (kernel 4.2.2) and LemLib 0.5.6.

## How it works

Start it with `MCL::init(numParticles)` (done in `initialize()`), then call `MCL::reset(startPose)` once the
robot's real starting pose is known; this sets the LemLib pose and re-scatters the particles around it.
Poses are in the field frame described below (origin at field centre).

Each MCL cycle (every 100 ms, in its own task):

1. **Read sensors** – the four V5 Distance Sensors (front, right, back, left) are read and converted to
   inches. Each reading is one of:
   - a distance (20–2000 mm; readings past 200 mm also need a confidence of at least 30/63),
   - *nothing in range* (the sensor's 9999 code), or
   - *invalid* (sensor error or unplugged, closer than 20 mm, or low confidence); this carries no information.
2. **Update** – the odometry change since the last cycle is taken from LemLib, converted into the
   robot frame (forward / right), and applied to every particle using that particle's own heading.
3. **Weigh** – for each particle, `fieldRepresentation::simulateCast` predicts what each sensor should read
   and at what angle the beam hits the surface. Each sensor is scored with a beam model:
   - **hit** – Gaussian around the predicted distance. σ combines the sensor spec (±15 mm, or 5%), model
     error, and extra spread for oblique hits.
   - **short** – something between the sensor and the modelled surface, such as a robot or game element.
     This bounds how much an obstruction can hurt a particle.
   - **random** – uniform noise.
   - **nothing in range** counts against a particle only if it expects a close (< 48"), nearly head-on
     wall, because the sensor also returns nothing on glancing hits.

   Particles off the field or inside a goal or loader get zero weight. Weighing is skipped until the robot
   has moved 0.5" or turned 1°, so a stationary robot doesn't keep reinforcing the same readings.
4. **Correct** – the weighted mean is applied to LemLib as a partial correction, but only when the
   cloud is tight (spread under 3"). Large jumps are held back unless they persist. The correction is
   excluded from the next cycle's odometry change, but motion LemLib tracked during the cycle is not.
5. **Resample** – only when the effective particle count drops below half the particle count.
   Resampling is low-variance (systematic): one random offset, then evenly spaced pointers through
   the cumulative weights, so particles are copied in proportion to their weight. All weights are
   then reset to `1 / N`. When the sensors stop matching, a few random particles are added within 12"
   of the odometry pose (not anywhere on the field, which is symmetric) so the filter can recover.

Each particle stores its heading as an offset from the IMU (`heading = imu rotation + headingOffset`),
so the filter can also correct IMU drift.

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

`telemetryData/` is a small React + TypeScript (Vite) page for checking MCL's pose estimates against
where the robot actually was. Upload a JSON log of poses, then step through them one frame at a time
with **Back** / **Forward**; the robot is drawn on a top-down 144" x 144" field with the pose printed
underneath.

The log is a JSON array of `[x, y, theta]` frames, in the same convention as the field model
(inches from field centre, heading in degrees, 0° = +Y, clockwise positive):

```json
[
  [-48, -48, 0],
  [-48, -36, 0],
  [-48, 48, 90]
]
```

[`telemetryData/test.json`](telemetryData/test.json) is a sample lap of the field. The intent is for
the robot to write this file to the SD card during a run; that logging isn't written yet.

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
| `telemetryData/` | Web page to step through logged poses on a drawing of the field |

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

- **Done** – field model and ray casting, the MCL task loop, sensor reading, effective particle
  count, and the telemetry viewer.
- **Needs review** – `MCL::resample` is written but doesn't compile yet (it uses `n` where the
  particle count is declared as `count`) and hasn't been tested.
- **TODO** – particle motion update (`MCL::update`), weighting (`MCL::weigh`), initial particle
  spread (`MCL::init`), writing the pose log to the SD card, and drawing the field elements and
  particles in the telemetry viewer.
