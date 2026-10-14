import React from 'react';

export interface DisplayFieldProps {
    position: number[]; 
    theta: number;     
}

export default function DisplayField({ position, theta }: DisplayFieldProps) {
    const FIELD_SIZE_PX = 600;
    const FIELD_SIZE_IN = 144; 
    const SCALE = FIELD_SIZE_PX / FIELD_SIZE_IN; 
    const CENTER_PX = FIELD_SIZE_PX / 2;

    const [xInches, yInches] = position;

    const pixelTop = CENTER_PX - (yInches * SCALE); 
    const pixelLeft = CENTER_PX + (xInches * SCALE);

    const robotSquare: React.CSSProperties = {
        width: '75px',
        height: '75px',
        backgroundColor: '#ffde49', 
        borderTop: '5px solid #ff7049',
        position: 'absolute',
        top: `${pixelTop}px`,
        left: `${pixelLeft}px`,
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

    return (
        <div>
            <div style={fieldSquare}>
                <div style={robotSquare}/>
            </div>
            <div style={{ marginTop: '10px', fontFamily: 'monospace' }}>
                x: {xInches.toFixed(2)} in, y: {yInches.toFixed(2)} in, theta: {theta.toFixed(2)}°
            </div>
        </div>
    );
}
