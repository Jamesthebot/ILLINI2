import React from 'react';
import { ParticleFrame, READING_INVALID, READING_NONE, SensorFrame } from './parseTelemetry';

export interface ParticlePanelProps {
    index: number;
    particle: ParticleFrame;
    particleCount: number;
    sensors: SensorFrame[];
    onClose: () => void;
}

function formatSimulated(value: number): string {
    if (isNaN(value)) return '—';
    if (value < 0) return 'no hit';
    return `${value.toFixed(2)}"`;
}

function formatMeasured(value: number | undefined): string {
    if (value === undefined || isNaN(value)) return '—';
    if (value === READING_NONE) return 'none';
    if (value <= READING_INVALID) return 'invalid';
    return `${value.toFixed(2)}"`;
}

function formatError(simulated: number, measured: number | undefined): string {
    if (measured === undefined || isNaN(simulated) || simulated < 0 || isNaN(measured) || measured < 0) return '—';
    const error = measured - simulated;
    return `${error >= 0 ? '+' : ''}${error.toFixed(2)}"`;
}

function formatWeight(weight: number, particleCount: number): string {
    if (isNaN(weight)) return '—';
    const relative = particleCount > 0 ? ` (${(weight * particleCount).toFixed(2)}× avg)` : '';
    return `${weight.toPrecision(4)}${relative}`;
}

const cell: React.CSSProperties = { padding: '4px 6px', borderBottom: '1px solid #ddd', textAlign: 'right' };
const headerCell: React.CSSProperties = { ...cell, borderBottom: '2px solid #999' };

export default function ParticlePanel({ index, particle, particleCount, sensors, onClose }: ParticlePanelProps) {
    return (
        <div style={{ width: '280px', flexShrink: 0, border: '1px solid #ccc', padding: '12px', fontFamily: 'monospace' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
                <strong>Particle {index}</strong>
                <button onClick={onClose}>Close</button>
            </div>
            <div style={{ marginBottom: '10px' }}>
                x: {particle.x.toFixed(2)} in, y: {particle.y.toFixed(2)} in, theta: {particle.theta.toFixed(2)}°
            </div>
            <table style={{ borderCollapse: 'collapse', width: '100%' }}>
                <thead>
                    <tr>
                        <th style={{ ...headerCell, textAlign: 'left' }}>Sensor</th>
                        <th style={headerCell}>Simulated</th>
                        <th style={headerCell}>Measured</th>
                        <th style={headerCell}>Error</th>
                    </tr>
                </thead>
                <tbody>
                    {particle.simulated.map((simulated, j) => (
                        <tr key={j}>
                            <td style={{ ...cell, textAlign: 'left' }}>
                                {j}{sensors[j] ? ` (${sensors[j].direction.toFixed(0)}°)` : ''}
                            </td>
                            <td style={cell}>{formatSimulated(simulated)}</td>
                            <td style={cell}>{formatMeasured(sensors[j]?.reading)}</td>
                            <td style={cell}>{formatError(simulated, sensors[j]?.reading)}</td>
                        </tr>
                    ))}
                    <tr>
                        <td style={{ ...cell, textAlign: 'left', fontWeight: 'bold' }}>Weight</td>
                        <td style={cell} colSpan={3}>{formatWeight(particle.weight, particleCount)}</td>
                    </tr>
                </tbody>
            </table>
        </div>
    );
}
