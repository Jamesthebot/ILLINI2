import React from 'react';
import Modal from './Modal';

export interface HowToUseProps {
    onClose: () => void;
    theme: React.CSSProperties;
}

export const code: React.CSSProperties = {
    background: '#1e1e1e',
    color: '#e0e0e0',
    padding: '12px',
    overflowX: 'auto',
    fontSize: '13px',
    lineHeight: 1.45,
};

const SETUP = `#include "telemetry.h"

telemetry* telemetryLog = nullptr;

void initialize() {
    // ...your existing setup...
    static telemetry log("/usd/telemetry.csv", 300, 4, 100);   // path, max particles, distance sensors, ms between frames
    telemetryLog = &log;
}`;

const LOOP = `telemetryLog->beginFrame(pose.x, pose.y, pose.theta, particles.size());

for (each distance sensor j, in a fixed order) {
    telemetryLog->writeSensor(readingInches,      // -1 = nothing in range, -2 = invalid
                              offsetX, offsetY,   // robot frame, inches: +x right, +y forward
                              directionDegrees);  // clockwise from the robot's front
}

for (each particle) {
    double simulated[4] = { /* expected reading of each sensor from this particle */ };
    telemetryLog->writeParticle(p.x, p.y, p.heading, p.weight, simulated, 4);
}

telemetryLog->endFrame();`;

const MCL = `static telemetry mclLog("/usd/telemetry.csv", 300, 4, MCL::LOOP_MS);
MCL::setTelemetry(&mclLog);
MCL::init(300);`;

const FORMAT = `MaxParticles,300
NumDistanceSensors,4
FramePeriodMs,100
X,Y,Theta,NumParticles,DistanceSensorReading0,DistanceSensorOffset0,DistanceSensorDirection0,...,ParticlePosition0,ParticleWeight0,SimulatedDistance0_0,...
12.31,-40.02,91.50,300,23.41,-1.00;1.75,0.00,...,12.20;-39.88;90.97,0.004127,23.10,...`;

export default function HowToUse({ onClose, theme }: HowToUseProps) {
    return (
        <Modal title="How to use" onClose={onClose} theme={theme}>

                <h4>1. Add the logger to your robot project</h4>
                <p>
                    Copy <code>include/telemetry.h</code> and <code>src/telemetry.cpp</code> from this repo into your PROS
                    project. Put a microSD card in the V5 Brain. Without one, the logger does nothing and the robot runs as usual.
                </p>

                <h4>2. Create the log once</h4>
                <p>
                    Create one <code>telemetry</code> object in <code>initialize()</code>. Make it <code>static</code> so it
                    isn't destroyed when <code>initialize()</code> returns, and keep a pointer to it for your loop. Creating it inside
                    <code> initialize()</code>, rather than as a global, means the file is opened after PROS has started. The constructor opens the file and writes the three
                    metadata lines and the column header. Pass the time between frames (your localization loop's period) as the
                    last argument. Running the program again overwrites the file.
                </p>
                <pre style={code}>{SETUP}</pre>

                <h4>3. Write one frame per localization step</h4>
                <p>
                    In your localization loop, call these in order. Write every sensor before the first particle, and keep the
                    sensors in the same order every frame. If you write fewer sensors or particles than the header allows, the
                    rest of the row is left blank.
                </p>
                <pre style={code}>{LOOP}</pre>
                <p>
                    Pose and particle positions are in inches from the field centre, with headings in degrees (0° = +Y, clockwise
                    positive), the same as LemLib. Simulated distances are what the sensor would read from that particle; use -1
                    for "no hit". The file is flushed every 10 frames, so the last second of data can be lost if the Brain loses power.
                </p>

                <h4>Using this repo's MCL</h4>
                <p>
                    <code>MCL</code> already writes a frame at the end of every step (every 100 ms). Pass it a log before starting it:
                </p>
                <pre style={code}>{MCL}</pre>

                <h4>4. View the run</h4>
                <p>
                    Take out the SD card, open <code>telemetry.csv</code> with the file picker on this page, and step through the
                    frames. Click a particle to compare its simulated distances with what the sensors measured.
                </p>

                <h4>File format</h4>
                <pre style={code}>{FORMAT}</pre>
                <p>
                    The first three lines tell the viewer how many particle and sensor columns to expect and how far apart frames
                    are, which sets the time shown and the playback speed. Logs without <code>FramePeriodMs</code> are treated as
                    100 ms per frame. Cells that hold more than one
                    number separate them with <code>;</code>: <code>DistanceSensorOffset</code> is <code>x;y</code> and
                    <code> ParticlePosition</code> is <code>x;y;theta</code>.
                </p>
        </Modal>
    );
}
