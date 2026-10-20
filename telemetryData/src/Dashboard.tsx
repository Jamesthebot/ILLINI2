import React, { useCallback, useEffect, useRef, useState } from 'react';
import DisplayField from './DisplayField';
import ParticlePanel from './ParticlePanel';
import HowToUse from './HowToUse';
import { DEFAULT_FRAME_PERIOD_MS, parseTelemetry, TelemetryFrame, TelemetryLog } from './parseTelemetry';

const EMPTY_FRAME: TelemetryFrame = { x: 0, y: 0, theta: 0, sensors: [], particles: [] };
const PLAYBACK_TICK_MS = 16;
const DEFAULT_ROBOT_SIZE_IN = 18;

export default function Dashboard() {
    const [telemetryLog, setTelemetryLog] = useState<TelemetryLog | null>(null);
    const [index, setIndex] = useState<number>(0);
    const [showRobot, setShowRobot] = useState<boolean>(true);
    const [showSensors, setShowSensors] = useState<boolean>(true);
    const [showParticles, setShowParticles] = useState<boolean>(true);
    const [selectedParticle, setSelectedParticle] = useState<number | null>(null);
    const [showHelp, setShowHelp] = useState<boolean>(false);
    const [robotWidth, setRobotWidth] = useState<number>(DEFAULT_ROBOT_SIZE_IN);
    const [robotLength, setRobotLength] = useState<number>(DEFAULT_ROBOT_SIZE_IN);
    const [playing, setPlaying] = useState<boolean>(false);
    const [playbackRate, setPlaybackRate] = useState<number>(1);
    const closeHelp = useCallback(() => setShowHelp(false), []);

    const maxIndex = telemetryLog ? Math.max(telemetryLog.frames.length - 1, 0) : 0;
    const framePeriodMs = telemetryLog?.framePeriodMs ?? DEFAULT_FRAME_PERIOD_MS;
    const formatSeconds = (frame: number) => (frame * framePeriodMs / 1000).toFixed(framePeriodMs % 100 === 0 ? 1 : 2);

    const indexRef = useRef(index);
    indexRef.current = index;
    const [playStart, setPlayStart] = useState<number>(0);

    useEffect(() => {
        if (!playing) return;
        const startIndex = indexRef.current;
        const startTime = performance.now();
        const timer = setInterval(() => {
            const target = Math.min(startIndex + Math.floor((performance.now() - startTime) * playbackRate / framePeriodMs), maxIndex);
            setIndex(target);
            if (target >= maxIndex) setPlaying(false);
        }, PLAYBACK_TICK_MS);
        return () => clearInterval(timer);
    }, [playing, playbackRate, maxIndex, framePeriodMs, playStart]);

    const handleFileUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
        const file = event.target.files?.[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (e) => {
            try {
                const parsed = parseTelemetry(e.target?.result as string);

                setTelemetryLog(parsed);
                setIndex(0);
                setPlaying(false);
                setSelectedParticle(null);
                console.log(`Parsed ${parsed.frames.length} frames (${parsed.maxParticles} max particles, ${parsed.numSensors} distance sensors)`);
            } catch (error) {
                alert("Failed to parse CSV. Please check file formatting.");
            }
        };
        reader.readAsText(file);
    };

    const play = () => {
        if (index >= maxIndex) setIndex(0);
        setPlaying(true);
    };

    const restart = () => {
        setIndex(0);
        setPlayStart(n => n + 1);
        setPlaying(true);
    };

    const step = (delta: number) => {
        setPlaying(false);
        setIndex(i => Math.min(Math.max(i + delta, 0), maxIndex));
    };

    const currentFrame = telemetryLog?.frames[index] ?? EMPTY_FRAME;
    const particle = selectedParticle !== null && showParticles ? currentFrame.particles[selectedParticle] : undefined;

    const checkbox = (label: string, checked: boolean, setChecked: (value: boolean) => void) => (
        <label style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
            <input type="checkbox" checked={checked} onChange={(e) => setChecked(e.target.checked)} />
            {label}
        </label>
    );

    const numberField = (label: string, value: number, setValue: (value: number) => void, min: number, stepSize: number, unit: string) => (
        <label style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
            {label}
            <input
                type="number"
                defaultValue={value}
                min={min}
                step={stepSize}
                style={{ width: '60px' }}
                onChange={(e) => {
                    const parsed = parseFloat(e.target.value);
                    if (!isNaN(parsed) && parsed >= min) setValue(parsed);
                }}
            />
            {unit}
        </label>
    );

    return (
        <div style={{ padding: '20px', fontFamily: 'sans-serif' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', margin: '0 0 15px 0' }}>
                <h3 style={{ margin: 0 }}>IVR MCL Debugger</h3>
                <button onClick={() => setShowHelp(true)}>How to Use</button>
            </div>

            <div style={{ marginBottom: '20px' }}>
                <input
                    type="file"
                    accept=".csv"
                    onChange={handleFileUpload}
                />
            </div>

            {telemetryLog && (
                <div style={{ marginBottom: '20px', display: 'flex', alignItems: 'center', gap: '10px', flexWrap: 'wrap' }}>
                    <button onClick={() => step(-1)} disabled={index === 0}>
                        Back
                    </button>
                    <span style={{ fontWeight: 'bold' }}>
                        Frame Index: {index} / {maxIndex} | {formatSeconds(index)} s / {formatSeconds(maxIndex)} s
                    </span>
                    <button onClick={() => step(1)} disabled={index === maxIndex}>
                        Forward
                    </button>
                    <button onClick={play} disabled={playing}>Play</button>
                    <button onClick={() => setPlaying(false)} disabled={!playing}>Pause</button>
                    <button onClick={restart}>Restart</button>
                    {numberField('Playback rate', playbackRate, setPlaybackRate, 0.1, 0.1, '×')}
                    <span style={{ marginLeft: '10px' }}>Particles: {currentFrame.particles.length}</span>
                </div>
            )}

            <div style={{ marginBottom: '15px', display: 'flex', gap: '16px', flexWrap: 'wrap' }}>
                {checkbox('Robot', showRobot, setShowRobot)}
                {checkbox('Distance sensors', showSensors, setShowSensors)}
                {checkbox('Particles', showParticles, setShowParticles)}
                {numberField('Robot width (x)', robotWidth, setRobotWidth, 1, 0.5, 'in')}
                {numberField('Robot length (y)', robotLength, setRobotLength, 1, 0.5, 'in')}
            </div>

            <div style={{ display: 'flex', alignItems: 'flex-start', gap: '20px' }}>
                <DisplayField
                    frame={currentFrame}
                    robotWidth={robotWidth}
                    robotLength={robotLength}
                    showRobot={showRobot}
                    showSensors={showSensors}
                    showParticles={showParticles}
                    selectedParticle={selectedParticle}
                    onSelectParticle={setSelectedParticle}
                />
                {particle && selectedParticle !== null && (
                    <ParticlePanel
                        index={selectedParticle}
                        particle={particle}
                        particleCount={currentFrame.particles.length}
                        sensors={currentFrame.sensors}
                        onClose={() => setSelectedParticle(null)}
                    />
                )}
            </div>

            {showHelp && <HowToUse onClose={closeHelp} />}
        </div>
    );
}
