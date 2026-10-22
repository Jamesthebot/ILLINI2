import React, { useCallback, useEffect, useRef, useState } from 'react';
import DisplayField from './DisplayField';
import ParticlePanel from './ParticlePanel';
import HowToUse from './HowToUse';
import SampleInfo from './SampleInfo';
import { DEFAULT_FRAME_PERIOD_MS, parseTelemetry, TelemetryFrame, TelemetryLog } from './parseTelemetry';

const EMPTY_FRAME: TelemetryFrame = { x: 0, y: 0, theta: 0, sensors: [], particles: [] };
const PLAYBACK_TICK_MS = 16;
const DEFAULT_ROBOT_SIZE_IN = 18;
const REPO_URL = 'https://github.com/brody-b9801/IVR-Localization';
const SAMPLE_CSV_NAME = 'sim_fixed_heading.csv';
const SAMPLE_CSV_URL = `./${SAMPLE_CSV_NAME}`;
const GITHUB_ICON_PATH = 'M8 0c4.42 0 8 3.58 8 8a8.013 8.013 0 0 1-5.45 7.59c-.4.08-.55-.17-.55-.38 0-.27.01-1.13.01-2.2 0-.75-.25-1.23-.54-1.48 1.78-.2 3.65-.88 3.65-3.95 0-.88-.31-1.59-.82-2.15.08-.2.36-1.02-.08-2.12 0 0-.67-.22-2.2.82-.64-.18-1.32-.27-2-.27-.68 0-1.36.09-2 .27-1.53-1.03-2.2-.82-2.2-.82-.44 1.1-.16 1.92-.08 2.12-.51.56-.82 1.28-.82 2.15 0 3.06 1.86 3.75 3.64 3.95-.23.2-.44.55-.51 1.07-.46.21-1.61.55-2.33-.66-.15-.24-.6-.83-1.23-.82-.67.01-.27.38.01.53.34.19.73.9.82 1.13.16.45.68 1.31 2.69.94 0 .67.01 1.3.01 1.49 0 .21-.15.45-.55.38A7.995 7.995 0 0 1 0 8c0-4.42 3.58-8 8-8Z';
const CONTENT_HEIGHT_PX =800;
const COLUMN_WIDTH_PX = 1000;
const SIDE_COLOR_LIGHT = '#eeeeee';
const SIDE_COLOR_DARK = '#0d0d0d';
const THEME_TRANSITION = 'background-color 0.3s ease, color 0.3s ease';

export default function Dashboard() {
    const [telemetryLog, setTelemetryLog] = useState<TelemetryLog | null>(null);
    const [index, setIndex] = useState<number>(0);
    const [showRobot, setShowRobot] = useState<boolean>(true);
    const [showSensors, setShowSensors] = useState<boolean>(true);
    const [showParticles, setShowParticles] = useState<boolean>(true);
    const [selectedParticle, setSelectedParticle] = useState<number | null>(null);
    const [showHelp, setShowHelp] = useState<boolean>(false);
    const [showSampleInfo, setShowSampleInfo] = useState<boolean>(false);    const closeSampleInfo = useCallback(() => setShowSampleInfo(false), []);

    const downloadSample = () => {
        const link = document.createElement('a');
        link.href = SAMPLE_CSV_URL;
        link.download = SAMPLE_CSV_NAME;
        link.click();
        setShowSampleInfo(true);
    };
    const [darkMode, setDarkMode] = useState<boolean>(false);
    const [zoom, setZoom] = useState<number>(() => window.innerHeight / CONTENT_HEIGHT_PX);

    useEffect(() => {
        const onResize = () => setZoom(window.innerHeight / CONTENT_HEIGHT_PX);
        window.addEventListener('resize', onResize);
        return () => window.removeEventListener('resize', onResize);
    }, []);
    const theme: React.CSSProperties = darkMode
        ? { background: '#1a1a1a', color: '#ffffff', colorScheme: 'dark' }
        : { background: '#ffffff', color: '#000000', colorScheme: 'light' };

    useEffect(() => {
        document.body.style.margin = '0';
        document.body.style.transition = THEME_TRANSITION;
        document.body.style.background = darkMode ? SIDE_COLOR_DARK : SIDE_COLOR_LIGHT;
    }, [darkMode]);
    const [robotWidth, setRobotWidth] = useState<number>(DEFAULT_ROBOT_SIZE_IN);
    const [robotLength, setRobotLength] = useState<number>(DEFAULT_ROBOT_SIZE_IN);
    const [playing, setPlaying] = useState<boolean>(false);
    const [playbackRate, setPlaybackRate] = useState<number>(1);
    const closeHelp = useCallback(() => setShowHelp(false), []);

    const maxIndex = telemetryLog ? Math.max(telemetryLog.frames.length - 1, 0) : 0;
    const framePeriodMs = telemetryLog?.framePeriodMs ?? DEFAULT_FRAME_PERIOD_MS;
    const formatSeconds = (frame: number) => (frame * framePeriodMs / 1000).toFixed(framePeriodMs % 100 === 0 ? 1 : 2);

    const fileInputRef = useRef<HTMLInputElement>(null);
    const [fileName, setFileName] = useState<string | null>(null);

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
        setFileName(file.name);

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

    const pageBreak: React.CSSProperties = { width: '100px', margin: '15px auto' };
    const rowStyle: React.CSSProperties ={ display: 'flex', justifyContent: 'center', gap: '16px', whiteSpace: 'nowrap' };

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
        <div style={{
            maxWidth: `${COLUMN_WIDTH_PX}px`, minHeight: `${CONTENT_HEIGHT_PX}px`, margin: '0 auto', boxSizing: 'border-box',
            padding: '20px', fontFamily: 'sans-serif', position: 'relative', textAlign: 'center', zoom, transition: THEME_TRANSITION, ...theme,
        }}>
            <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '12px', marginBottom: '8px' }}>
                <h1 style={{ margin: 0 }}>IVR MCL Debugger</h1>
                <hr style={{ ...pageBreak, margin: '3px auto' }} />
                <input
                    ref={fileInputRef}
                    type="file"
                    accept=".csv"
                    onChange={handleFileUpload}
                    style={{ display: 'none' }}
                />
                <button onClick={() => fileInputRef.current?.click()}>Choose File</button>
                <span>{fileName ?? 'No file chosen'}</span>
            </div>

            {telemetryLog && (
                <div style={{ marginBottom: '20px', display: 'flex', justifyContent: 'center', alignItems: 'center', gap: '10px', flexWrap: 'wrap' }}>
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

            <hr style={pageBreak} />
            <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '10px' }}>
                <div style={rowStyle}>
                    {checkbox('Robot', showRobot, setShowRobot)}
                    {checkbox('Distance sensors', showSensors, setShowSensors)}
                    {checkbox('Particles', showParticles, setShowParticles)}
                </div>
                <div style={rowStyle}>
                    {numberField('Robot width (x)', robotWidth, setRobotWidth, 1, 0.5, 'in')}
                    {numberField('Robot length (y)', robotLength, setRobotLength, 1, 0.5, 'in')}
                </div>
                <div style={rowStyle}>
                    {checkbox('Dark mode', darkMode, setDarkMode)}
                </div>
            </div>
            <hr style={pageBreak} />

            <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'flex-start', gap: '20px' }}>
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

            <hr style={pageBreak} />
            <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', gap: '10px', flexWrap: 'wrap' }}>
                <button onClick={() => setShowHelp(true)}>How to Use</button>
                <button
                    onClick={() => window.open(REPO_URL, '_blank', 'noopener')}
                    title="GitHub repository"
                    aria-label="GitHub repository"
                >
                    <svg width="20" height="20" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true">
                        <path d={GITHUB_ICON_PATH} />
                    </svg>
                </button>
                <button onClick={downloadSample}>Download test CSV</button>
            </div>

            {showHelp && <HowToUse onClose={closeHelp} theme={theme} />}
            {showSampleInfo && <SampleInfo onClose={closeSampleInfo} theme={theme} />}
        </div>
    );
}
