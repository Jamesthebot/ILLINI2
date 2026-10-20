export type FieldElementKind = 'loader' | 'goal';
export type Alliance = 'red' | 'blue' | 'neutral';

export interface FieldElement {
    kind: FieldElementKind;
    alliance: Alliance;
    name: string;
    min: [number, number];
    max: [number, number];
}

export const FIELD_HALF_WIDTH_IN = 70.1968;

export const FIELD_ELEMENTS: FieldElement[] = [
    { kind: 'loader', alliance: 'blue', name: 'Blue loader', min: [66.4645, 56.346], max: [70.1968, 61.1804] },
    { kind: 'loader', alliance: 'red', name: 'Red loader', min: [-70.1968, -61.1804], max: [-66.4645, -56.346] },
    { kind: 'loader', alliance: 'blue', name: 'Blue loader', min: [66.4645, -61.1804], max: [70.1968, -56.346] },
    { kind: 'loader', alliance: 'red', name: 'Red loader', min: [-70.1968, 56.346], max: [-66.4645, 61.1804] },

    { kind: 'goal', alliance: 'neutral', name: 'Neutral Goal 1', min: [-49.8988, 20.738], max: [-44.2836, 26.3532] },
    { kind: 'goal', alliance: 'neutral', name: 'Neutral Goal 4', min: [-26.3532, 44.2836], max: [-20.738, 49.8988] },
    { kind: 'goal', alliance: 'blue', name: 'Blue 2', min: [44.2856, 20.74], max: [49.8968, 26.3512] },
    { kind: 'goal', alliance: 'red', name: 'Red 2', min: [-49.8968, -26.3512], max: [-44.2856, -20.74] },
    { kind: 'goal', alliance: 'blue', name: 'Blue 3', min: [20.74, 44.2856], max: [26.3512, 49.8968] },
    { kind: 'goal', alliance: 'neutral', name: 'Neutral Goal 0', min: [-2.8076, -2.8076], max: [2.8076, 2.8076] },
    { kind: 'goal', alliance: 'red', name: 'Red 3', min: [-26.3512, -49.8968], max: [-20.74, -44.2856] },
    { kind: 'goal', alliance: 'neutral', name: 'Neutral Goal 4', min: [20.738, -49.8988], max: [26.3532, -44.2836] },
    { kind: 'goal', alliance: 'neutral', name: 'Neutral Goal 1', min: [44.2836, -26.3532], max: [49.8988, -20.738] },
];
