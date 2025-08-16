import { musicNotes } from './commands.js';

export function parseMidi(arrayBuffer) {
    const data = new DataView(arrayBuffer);
    let offset = 0;

    function readUint32() {
        if (offset + 4 <= data.byteLength) {
            const value = data.getUint32(offset);
            offset += 4;
            return value;
        }
        return 0;
    }
    
    function readUint16() {
        if (offset + 2 <= data.byteLength) {
            const value = data.getUint16(offset);
            offset += 2;
            return value;
        }
        return 0;
    }
    
    function readUint8() {
        if (offset + 1 <= data.byteLength) {
            const value = data.getUint8(offset);
            offset += 1;
            return value;
        }
        return 0;
    }
    
    function readVarLength() {
        let value = 0;
        let byte;
        do {
            byte = readUint8();
            value = (value << 7) | (byte & 0x7F);
        } while (byte & 0x80);
        return value;
    }
    
    // Read header chunk
    const headerChunkType = readUint32();
    const headerChunkLength = readUint32();
    const formatType = readUint16();
    const numberOfTracks = readUint16();
    const ticksPerBeat = readUint16();
    
    const notes = [];
    
    for (let trackIndex = 0; trackIndex < numberOfTracks; trackIndex++) {
        // Read track chunk
        const trackChunkType = readUint32();
        const trackChunkLength = readUint32();
        const trackEnd = offset + trackChunkLength;
    
        let currentTime = 0;
    
        while (offset < trackEnd) {
            const deltaTime = readVarLength();
            currentTime += deltaTime;
    
            if (offset >= data.byteLength) break; // Ensure we do not read beyond the end of the DataView
    
            const eventType = readUint8();
    
            if (eventType === 0xFF) {
                // Meta event
                const metaType = readUint8();
                const metaLength = readVarLength();
                if (offset + metaLength > data.byteLength) throw new RangeError("Offset is outside the bounds of the DataView for meta event");
                offset += metaLength;
            } else if (eventType === 0xF0 || eventType === 0xF7) {
                // Sysex event
                const sysexLength = readVarLength();
                if (offset + sysexLength > data.byteLength) throw new RangeError("Offset is outside the bounds of the DataView for sysex event");
                offset += sysexLength;
            } else {
                // MIDI event
                const midiEventType = eventType >> 4;
                const channel = eventType & 0x0F;
    
                if (midiEventType === 0x9) {
                    // Note on event
                    const note = readUint8();
                    const velocity = readUint8();
                    if (velocity > 0) {
                        const duration = Math.max(Math.round(currentTime * 64 / ticksPerBeat), 1); // Convert ticks to 1/64th of a second, ensure duration is at least 1
                        currentTime = 0; // Reset currentTime after calculating duration
                        const roombaNote = Object.values(musicNotes).find(value => value === note);
                        if (roombaNote !== undefined) {
                            notes.push({
                                note: roombaNote,
                                duration: duration
                            });
                        }
                    }
                } else if (midiEventType === 0x8) {
                    // Note off event
                    if (offset + 2 > data.byteLength) throw new RangeError("Offset is outside the bounds of the DataView for note off event");
                    offset += 2; // Skip note and velocity
                } else {
                    // Other MIDI events
                    if (offset + 1 > data.byteLength) throw new RangeError("Offset is outside the bounds of the DataView for other MIDI events");
                    offset += 1; // Skip the next byte
                    if (midiEventType === 0xC || midiEventType === 0xD) {
                        // Program change or channel pressure
                        // These events have only one data byte
                    } else {
                        // Other events have two data bytes
                        if (offset + 1 > data.byteLength) throw new RangeError("Offset is outside the bounds of the DataView for other MIDI events with two data bytes");
                        offset += 1;
                    }
                }
            }
        }
    }
    
    return notes;
}