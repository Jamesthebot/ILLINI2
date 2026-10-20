import React from 'react';
import Modal from './Modal';
import { code } from './HowToUse';

export interface SampleInfoProps {
    onClose: () => void;
    theme: React.CSSProperties;
}

const BUILD = `cmake -S tools/mcl_sim -B tools/mcl_sim/build
cmake --build tools/mcl_sim/build --config Release
tools/mcl_sim/build/Release/mcl_sim.exe tools/mcl_sim/out/sim.csv`;

export default function SampleInfo({ onClose, theme }: SampleInfoProps) {
    return (
        <Modal title="Sample run" onClose={onClose} theme={theme}>
            <p>
                <code>sim_fixed_heading.csv</code> is downloading. It's a log from running this repo's real MCL code on a PC
                against a simulated robot, written in the same format the robot writes to the SD card.
            </p>

            <h4>Watch it</h4>
            <p>
                Open the file with <strong>Choose File</strong>, then press <strong>Play</strong> or step with
                <strong> Back</strong> / <strong>Forward</strong>. Click a particle to see its weight and how its simulated
                distances compare with the sensor readings.
            </p>

            <h4>What happens during the run</h4>
            <ul>
                <li>The robot starts at (-34, -34) facing +X and drives two laps of a square at ±34", turning in place at each
                    corner. It steers by its odometry, the way a LemLib motion would.</li>
                <li>Odometry reads 2% long, with a little noise and sideways slip, and the IMU drifts 0.1°/s, so odometry
                    slowly drifts away from where the robot really is.</li>
                <li>Each distance sensor reads the distance from the robot's true pose to the field model, plus noise at the
                    V5 sensor's spec. A few readings are short (as if something were in the way) or low-confidence, and glancing
                    or out-of-range hits read as nothing in range.</li>
                <li>MCL runs every 100 ms with 300 particles and nudges odometry toward its estimate.</li>
                <li>At 12 s (around frame 120) the robot is pushed 4" toward +Y, and odometry doesn't see it. The sensor rays
                    stop lining up with the walls and the particle cloud spreads, then MCL pulls odometry back over about 3 s.</li>
            </ul>
            <p>
                The robot drawn is odometry after MCL's corrections; the true pose isn't in the log. Over the run, the position
                error is 0.65" on average with MCL, against 2.31" for odometry alone, and the heading drift is corrected too.
                The simulated sensors use the same field model as MCL, so real runs will be noisier.
            </p>

            <h4>Run it yourself</h4>
            <p>
                The simulator is in <code>tools/mcl_sim/</code>. It needs CMake and a C++ compiler (Visual Studio with the C++
                workload on Windows). From the repo root:
            </p>
            <pre style={code}>{BUILD}</pre>
            <p>
                It prints the error over time and writes the CSV. To try a change to MCL without editing <code>src/</code>,
                configure with <code>-DMCL_SOURCE=path/to/mcl.cpp</code>. To try a different loop period, configure a separate
                build folder with <code>-DMCL_LOOP_MS=50</code>. On Linux or macOS the program is
                <code> build/mcl_sim</code>.
            </p>
        </Modal>
    );
}
