import { useState } from 'react';
import DisplayField from './DisplayField'; 

interface RobotData {
    [index: number]: number[]; 
}

export default function Dashboard() {
    const [robotData, setRobotData] = useState<RobotData | null>(null);
    const [index, setIndex] = useState<number>(0);
    
    const maxIndex = robotData ? Object.keys(robotData).length - 1 : 0;

    const handleFileUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
        const file = event.target.files?.[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (e) => {
            try {
                const parsedCSV = parseCSV(e.target?.result as string);
                
                setRobotData(parsedCSV);
                setIndex(0); 
                console.log("Successfully parsed robot map data:", parsedCSV);
            } catch (error) {
                alert("Failed to parse CSV. Please check file formatting.");
            }
        };
        reader.readAsText(file);
    };

    function parseCSV(csv : string) : any {
        const lineRegExp = /[^\r\n]+/g;
        let match;
        let rowIndex : number = 0;
        const data : RobotData = {}
        while ((match = lineRegExp.exec(csv)) !== null) {
            if (rowIndex === 0) {
                rowIndex++;
                continue;
            } 
            const row: string = match[0];
            const values : number[] = row.split(',').map(str => parseFloat(str));
            data[rowIndex - 1] = values;
            rowIndex++;
        }
        return data;
    }

    const currentWaypoint = robotData?.[index] || [0,0,0];
    
    const currentPosition = [currentWaypoint[0], currentWaypoint[1]];
    const currentTheta = currentWaypoint[2];

    return (
        <div style={{ padding: '20px', fontFamily: 'sans-serif' }}>
            <h3 style={{ margin: '0 0 15px 0' }}>IVR MCL Debugger</h3>
            
            <div style={{ marginBottom: '20px' }}>
                <input 
                    type="file" 
                    accept=".csv" 
                    onChange={handleFileUpload} 
                />
            </div>

            {robotData && (
                <div style={{ marginBottom: '20px', display: 'flex', alignItems: 'center', gap: '10px' }}>
                    <button onClick={() => index > 0 && setIndex(p => p - 1)} disabled={index === 0}>
                        Back
                    </button>
                    <span style={{ fontWeight: 'bold' }}>Frame Index: {index} / {maxIndex}</span>
                    <button onClick={() => index < maxIndex && setIndex(p => p + 1)} disabled={index === maxIndex}>
                        Forward
                    </button>
                </div>
            )}

            <DisplayField position={currentPosition} theta={currentTheta} />
        </div>
    );
}
