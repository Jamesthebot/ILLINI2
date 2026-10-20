import React from 'react';
import { MAX_SENSOR_RANGE_IN, READING_INVALID, READING_NONE, TelemetryFrame } from './parseTelemetry';
import { Alliance, FIELD_ELEMENTS, FIELD_HALF_WIDTH_IN } from './fieldElements';

export interface DisplayFieldProps {
    frame: TelemetryFrame;
    robotWidth: number;
    robotLength: number;
    showRobot: boolean;
    showSensors: boolean;
    showParticles: boolean;
    selectedParticle: number | null;
    onSelectParticle: (index: number | null) => void;
}

const FIELD_SIZE_PX = 600;
const FIELD_SIZE_IN = 144;
const SCALE = FIELD_SIZE_PX / FIELD_SIZE_IN;
const CENTER_PX = FIELD_SIZE_PX / 2;
const PARTICLE_RADIUS_PX = 3;
const PARTICLE_HEADING_IN = 4;
const SENSOR_COLOR = '#4fc3f7';
const SELECTED_COLOR = '#ff00ff';
const UNWEIGHTED_COLOR = '#e0e0e0';
const ALLIANCE_COLORS: Record<Alliance, string> = { red: '#e53935', blue: '#1e88e5', neutral: '#bdbdbd' };

function toPx(x: number, y: number): [number, number] {
    return [CENTER_PX + x * SCALE, CENTER_PX - y * SCALE];
}

function headingVector(degrees: number): [number, number] {
    const rad = degrees * Math.PI / 180;
    return [Math.sin(rad), Math.cos(rad)];
}

function sensorLabel(reading: number): string {
    if (reading === READING_NONE) return 'none';
    if (reading <= READING_INVALID) return 'invalid';
    return `${reading.toFixed(1)}"`;
}

export default function DisplayField({ frame, robotWidth, robotLength, showRobot, showSensors, showParticles, selectedParticle, onSelectParticle }: DisplayFieldProps) {
    const { x: xInches, y: yInches, theta } = frame;
    const [pixelLeft, pixelTop] = toPx(xInches, yInches);

    const robotSquare: React.CSSProperties = {
        width: `${robotWidth * SCALE}px`,
        height: `${robotLength * SCALE}px`,
        boxSizing: 'border-box',
        backgroundColor: '#ffde49',
        borderTop: '5px solid #ff7049',
        position: 'absolute',
        top: `${pixelTop}px`,
        left: `${pixelLeft}px`,
        opacity: 0.75,
        transform: `translate(-50%, -50%) rotate(${theta}deg)`,
        transformOrigin: 'center center',
        transition: 'transform 0.1s ease, top 0.1s ease, left 0.1s ease',
    };

    const fieldSquare: React.CSSProperties = {
        width: `${FIELD_SIZE_PX}px`,
        height: `${FIELD_SIZE_PX}px`,
        backgroundColor: '#616161',
        position: 'relative',
        overflow: 'hidden'
    };

    const [wallLeft, wallTop] = toPx(-FIELD_HALF_WIDTH_IN, FIELD_HALF_WIDTH_IN);
    const wallSize = 2 * FIELD_HALF_WIDTH_IN * SCALE;
    const fieldMarks = (
        <g pointerEvents="none">
            <rect x={wallLeft} y={wallTop} width={wallSize} height={wallSize} fill="none" stroke="#9e9e9e" strokeWidth={3} />
            {FIELD_ELEMENTS.map((element, i) => {
                const [left, top] = toPx(element.min[0], element.max[1]);
                const [right, bottom] = toPx(element.max[0], element.min[1]);
                const color = ALLIANCE_COLORS[element.alliance];
                return (
                    <rect key={i} x={left} y={top} width={right - left} height={bottom - top}
                        fill={color} fillOpacity={element.kind === 'goal' ? 0.85 : 0.5}
                        stroke={color} strokeWidth={1.5} />
                );
            })}
        </g>
    );

    const [sinT, cosT] = headingVector(theta);
    const sensorLines = frame.sensors.map((sensor, j) => {
        const startX = xInches + sensor.offsetX * cosT + sensor.offsetY * sinT;
        const startY = yInches + sensor.offsetY * cosT - sensor.offsetX * sinT;
        const [dx, dy] = headingVector(theta + sensor.direction);
        const invalid = sensor.reading <= READING_INVALID;
        const none = sensor.reading === READING_NONE;
        const length = invalid ? 0 : none ? MAX_SENSOR_RANGE_IN : sensor.reading;

        const [x1, y1] = toPx(startX, startY);
        const [x2, y2] = toPx(startX + dx * length, startY + dy * length);
        const [labelX, labelY] = invalid ? [x1, y1] : [(x1 + x2) / 2, (y1 + y2) / 2];

        return (
            <g key={j}>
                {invalid ? (
                    <circle cx={x1} cy={y1} r={3} fill="none" stroke={SENSOR_COLOR} strokeWidth={1.5} />
                ) : (
                    <line x1={x1} y1={y1} x2={x2} y2={y2} stroke={SENSOR_COLOR} strokeWidth={2}
                        strokeDasharray={none ? '6 4' : undefined} opacity={none ? 0.5 : 1} />
                )}
                <text x={labelX} y={labelY - 6} textAnchor="middle" fontSize={12} fontFamily="monospace"
                    fill="#ffffff" stroke="#000000" strokeWidth={3} paintOrder="stroke">
                    {sensorLabel(sensor.reading)}
                </text>
            </g>
        );
    });

    const weights = frame.particles.map(p => p.weight).filter(w => !isNaN(w));
    const minWeight = Math.min(...weights);
    const maxWeight = Math.max(...weights);
    const weightColor = (weight: number) => {
        if (isNaN(weight)) return UNWEIGHTED_COLOR;
        const t = maxWeight > minWeight ? (weight - minWeight) / (maxWeight - minWeight) : 0.5;
        return `hsl(${120 * t}, 85%, 50%)`;
    };

    const particleMarks = frame.particles.map((particle, i) => {
        const [px, py] = toPx(particle.x, particle.y);
        const [dx, dy] = headingVector(particle.theta);
        const selected = i === selectedParticle;
        const color = selected ? SELECTED_COLOR : weightColor(particle.weight);
        const headingPx = PARTICLE_HEADING_IN * SCALE;
        return (
            <g key={i} style={{ cursor: 'pointer' }}
                onClick={(event) => { event.stopPropagation(); onSelectParticle(selected ? null : i); }}>
                <line x1={px} y1={py} x2={px + dx * headingPx} y2={py - dy * headingPx} stroke={color} strokeWidth={1} />
                <circle cx={px} cy={py} r={selected ? PARTICLE_RADIUS_PX * 1.7 : PARTICLE_RADIUS_PX}
                    fill={color} stroke="#212121" strokeWidth={0.75} />
            </g>
        );
    });
    const drawOrder = frame.particles
        .map((particle, i) => ({ i, weight: isNaN(particle.weight) ? 0 : particle.weight }))
        .filter(({ i }) => i !== selectedParticle)
        .sort((a, b) => a.weight - b.weight)
        .map(({ i }) => particleMarks[i]);
    const selectedMark = selectedParticle !== null ? particleMarks[selectedParticle] : null;

    return (
        <div>
            <div style={fieldSquare}>
                <svg width={FIELD_SIZE_PX} height={FIELD_SIZE_PX} style={{ position: 'absolute', top: 0, left: 0 }}>
                    {fieldMarks}
                </svg>
                {showRobot && <div style={robotSquare}/>}
                <svg width={FIELD_SIZE_PX} height={FIELD_SIZE_PX} style={{ position: 'absolute', top: 0, left: 0 }}
                    onClick={() => onSelectParticle(null)}>
                    {showParticles && drawOrder}
                    {showParticles && selectedMark}
                    {showSensors && sensorLines}
                </svg>
            </div>
            <div style={{ marginTop: '10px', fontFamily: 'monospace' }}>
                x: {xInches.toFixed(2)} in, y: {yInches.toFixed(2)} in, theta: {theta.toFixed(2)}°
            </div>
        </div>
    );
}
