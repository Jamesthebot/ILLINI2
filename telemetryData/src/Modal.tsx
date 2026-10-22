import React, { useEffect } from 'react';

export interface ModalProps {
    title: string;
    onClose: () => void;
    theme: React.CSSProperties;
    children: React.ReactNode;
}

export default function Modal({ title, onClose, theme, children }: ModalProps) {
    useEffect(() => {
        const onKey = (event: KeyboardEvent) => { if (event.key === 'Escape') onClose(); };
        window.addEventListener('keydown', onKey);
        return () => window.removeEventListener('keydown', onKey);
    }, [onClose]);

    return (
        <div onClick={onClose} style={{
            position: 'fixed', inset: 0, background: 'rgba(0, 0, 0, 0.5)',
            display: 'flex', justifyContent: 'center', alignItems: 'flex-start', padding: '40px 16px', overflowY: 'auto', zIndex: 10,
        }}>
            <div onClick={(event) => event.stopPropagation()} style={{
                ...theme, border: '1px solid #888888', padding: '24px', maxWidth: '760px', width: '100%', textAlign: 'left',
            }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                    <h3 style={{ margin: 0 }}>{title}</h3>
                    <button onClick={onClose}>Close</button>
                </div>
                {children}
            </div>
        </div>
    );
}
