export const READING_NONE = -1;
export const READING_INVALID = -2;
export const MAX_SENSOR_RANGE_IN = 78.74;

export interface SensorFrame {
    reading: number;
    offsetX: number;
    offsetY: number;
    direction: number;
}

export interface ParticleFrame {
    x: number;
    y: number;
    theta: number;
    weight: number;
    simulated: number[];
}

export interface TelemetryFrame {
    x: number;
    y: number;
    theta: number;
    sensors: SensorFrame[];
    particles: ParticleFrame[];
}

export interface TelemetryLog {
    maxParticles: number;
    numSensors: number;
    frames: TelemetryFrame[];
}

function isMetadataLine(cells: string[]): boolean {
    return cells.length === 2 && cells[0].trim() !== '' && isNaN(Number(cells[0])) && cells[1].trim() !== '' && !isNaN(Number(cells[1]));
}

function parts(cell: string | undefined): number[] | null {
    if (cell === undefined || cell.trim() === '') return null;
    const values = cell.split(';').map(Number);
    return values.some(isNaN) ? null : values;
}

export function parseTelemetry(csv: string): TelemetryLog {
    const lines = csv.split(/\r?\n/).filter(line => line.trim() !== '');

    const metadata = new Map<string, number>();
    let lineIndex = 0;
    while (lineIndex < lines.length && isMetadataLine(lines[lineIndex].split(','))) {
        const [key, value] = lines[lineIndex].split(',');
        metadata.set(key.trim().toLowerCase(), Number(value));
        lineIndex++;
    }
    const maxParticles = Math.max(0, metadata.get('maxparticles') ?? 0);
    const numSensors = Math.max(0, metadata.get('numdistancesensors') ?? 0);

    if (lineIndex >= lines.length) throw new Error('Missing CSV header');
    const header = lines[lineIndex++].split(',').map(name => name.trim().toLowerCase());
    const column = new Map<string, number>();
    header.forEach((name, i) => column.set(name, i));
    const col = (name: string) => column.get(name.toLowerCase()) ?? -1;

    const xCol = col('X'), yCol = col('Y'), thetaCol = col('Theta');
    if (xCol < 0 || yCol < 0 || thetaCol < 0) throw new Error('CSV header needs X, Y and Theta columns');
    const numParticlesCol = col('NumParticles');

    const sensorCols = Array.from({ length: numSensors }, (_, j) => ({
        reading: col(`DistanceSensorReading${j}`),
        offset: col(`DistanceSensorOffset${j}`),
        direction: col(`DistanceSensorDirection${j}`),
    }));
    const particleCols = Array.from({ length: maxParticles }, (_, i) => ({
        position: col(`ParticlePosition${i}`),
        weight: col(`ParticleWeight${i}`),
        simulated: Array.from({ length: numSensors }, (_, j) => col(`SimulatedDistance${i}_${j}`)),
    }));

    const frames: TelemetryFrame[] = [];
    for (; lineIndex < lines.length; lineIndex++) {
        const cells = lines[lineIndex].split(',');
        const num = (i: number) => (i < 0 || cells[i] === undefined || cells[i].trim() === '' ? NaN : Number(cells[i]));

        const x = num(xCol), y = num(yCol), theta = num(thetaCol);
        if (isNaN(x) || isNaN(y) || isNaN(theta)) continue;

        const sensors: SensorFrame[] = [];
        for (const c of sensorCols) {
            const offset = parts(cells[c.offset]);
            const reading = num(c.reading);
            const direction = num(c.direction);
            if (!offset || offset.length < 2 || isNaN(direction)) continue;
            sensors.push({ reading: isNaN(reading) ? READING_INVALID : reading, offsetX: offset[0], offsetY: offset[1], direction });
        }

        const declared = numParticlesCol < 0 ? maxParticles : num(numParticlesCol);
        const numParticles = Math.min(isNaN(declared) ? 0 : declared, maxParticles);
        const particles: ParticleFrame[] = [];
        for (let i = 0; i < numParticles; i++) {
            const position = parts(cells[particleCols[i].position]);
            if (!position || position.length < 3) continue;
            particles.push({
                x: position[0],
                y: position[1],
                theta: position[2],
                weight: num(particleCols[i].weight),
                simulated: particleCols[i].simulated.map(num),
            });
        }

        frames.push({ x, y, theta, sensors, particles });
    }

    return { maxParticles, numSensors, frames };
}
