import { sendWSCommand } from './websocket.js';
import { afterRoombaShutdown, afterRoombaTurnedOn } from './main.js';
import { SensorPacketId } from './sensors.js';
import { state, toggleRoombaDataPause } from './main.js';
import { parseMidi } from './midiParser.js';

export const CommandCode = {
    RESET: "reset",
    START: "start",
    BAUD: "baud",
    SAFE: "safe",
    FULL: "full",
    POWER: "power",
    SPOT: "spot",
    CLEAN: "clean",
    MAX: "max",
    DRIVE: "drive",
    MOTORS: "motors",
    LEDS: "leds",
    SONG: "song",
    PLAY: "play",
    SENSORS: "sensors",
    SEEK_DOCK: "seek_dock",
    PWM_MOTORS: "pwm_motors",
    DRIVE_DIRECT: "drive_direct",
    DRIVE_PWM: "drive_pwm",
    QUERY_LIST: "query_list",
    STREAM: "stream",
    PAUSE_RESUME_STREAM: "pause_resume_stream",
    SCHEDULING_LEDS: "scheduling_leds",
    DIGIT_LEDS_RAW: "digit_leds_raw",
    DIGIT_LEDS_ASCII: "digit_leds_ascii",
    BUTTONS: "buttons",
    SCHEDULE: "schedule",
    SET_DAY_TIME: "set_day_time",
    STOP: "stop",
    WAKEUP: "wakeup", // Custom commands
    STREAM_SONG: "stream_song"
};

const CommandInfo = {
    reset: { opcode: 7, dataBytes: 0 },
    start: { opcode: 128, dataBytes: 0 },
    baud: { opcode: 129, dataBytes: 1, 
        send: (commandDiv) => {
            const baudRate = commandDiv.querySelector('select').value;
            sendCommand(CommandCode.BAUD, baudRate);
        }
    },
    safe: { opcode: 131, dataBytes: 0 },
    full: { opcode: 132, dataBytes: 0 },
    power: { opcode: 133, dataBytes: 0 },
    spot: { opcode: 134, dataBytes: 0 },
    clean: { opcode: 135, dataBytes: 0 },
    max: { opcode: 136, dataBytes: 0 },
    drive: { opcode: 137, dataBytes: 2,
        send: (commandDiv) => {
            const inputs = commandDiv.querySelectorAll('input[type="number"]');
            const velocity = inputs[0].value | 0;
            const radius = inputs[1].value | 0;
            sendCommand(CommandCode.DRIVE, velocity, radius);
        }
    },
    motors: { opcode: 138, dataBytes: 1,
        send: (commandDiv) => {
            const motorSelect = commandDiv.querySelector('select');
            let motors = 0;
            for (const option of motorSelect.selectedOptions) {
                motors |= 1 << parseInt(option.value);
            }
            sendCommand(CommandCode.MOTORS, motors);
        }
    },
    leds: { opcode: 139, dataBytes: 3,
        send: (commandDiv) => {
            const ledSelect = commandDiv.querySelector('select');
            const inputs = commandDiv.querySelectorAll('input[type="number"]');
            const color = inputs[0].value | 0;
            const intensity = inputs[1].value | 0;
            let leds = 0;
            for (const option of ledSelect.selectedOptions) {
                leds |= 1 << parseInt(option.value);
            }
            sendCommand(CommandCode.LEDS, leds, color, intensity);
        }
    },
    song: { opcode: 140, dataBytes: -2,
        send: (commandDiv) => {
            const selects = commandDiv.querySelectorAll('select');
            const songSelect = selects[0];
            const selectNotes = commandDiv.querySelectorAll('select.note');
            const selectDurations = commandDiv.querySelectorAll('input[type="number"]');
            let songLength = 0;
            let songData = [];
            
            for (let i = 0; i < selectNotes.length; i++) {
                songData.push(parseInt(selectNotes[i].value));
                songData.push(parseInt(selectDurations[i].value | 0));
                songLength++;
            }

            while ((songData[songData.length - 1] === 0 || songData[songData.length - 2] === 0) && songLength > 1) {
                songData.pop();
                songData.pop();
                songLength--;
            }

            sendCommand(CommandCode.SONG, parseInt(songSelect.value), songLength, ...songData);
        }
    },
    play: { opcode: 141, dataBytes: 1,
        send: (commandDiv) => {
            const songSelect = commandDiv.querySelector('select');
            sendCommand(CommandCode.PLAY, parseInt(songSelect.value));
        }
    },
    sensors: { opcode: 142, dataBytes: 1,
        send: (commandDiv) => {
            const sensorSelect = commandDiv.querySelector('select');
            sendCommand(CommandCode.SENSORS, parseInt(sensorSelect.value));
        }
    },
    seek_dock: { opcode: 143, dataBytes: 0 },
    pwm_motors: { opcode: 144, dataBytes: 3,
        send: (commandDiv) => {
            const inputs = commandDiv.querySelectorAll('input[type="number"]');
            const mainBrushPWM = inputs[0].value | 0;
            const sideBrushPWM = inputs[1].value | 0;
            const vacuumPWM = inputs[2].value | 0;
            sendCommand(CommandCode.PWM_MOTORS, mainBrushPWM, sideBrushPWM, vacuumPWM);
        }
    },
    drive_direct: { opcode: 145, dataBytes: 2,
        send: (commandDiv) => {
            const inputs = commandDiv.querySelectorAll('input[type="number"]');
            const rightVelocity = inputs[0].value | 0;
            const leftVelocity = inputs[1].value | 0;
            sendCommand(CommandCode.DRIVE_DIRECT, rightVelocity, leftVelocity);
        }
    },
    drive_pwm: { opcode: 146, dataBytes: 2,
        send: (commandDiv) => {
            const inputs = commandDiv.querySelectorAll('input[type="number"]');
            const rightPWM = inputs[0].value | 0;
            const leftPWM = inputs[1].value | 0;
            sendCommand(CommandCode.DRIVE_PWM, rightPWM, leftPWM);
        }
    },
    stream: { opcode: 148, dataBytes: -1,
        send: (commandDiv) => {
            const nPackets = commandDiv.querySelector('.n-packets').value | 0;
            const selects = commandDiv.querySelectorAll('.sensor-select');
            let sensorPacketIds = [];
            for (const select of selects) {
                sensorPacketIds.push(select.value | 0);
            }
            sendCommand(CommandCode.STREAM, nPackets, ...sensorPacketIds);
        }
    },
    query_list: { opcode: 149, dataBytes: -1,
        send: (commandDiv) => {
            const nPackets = commandDiv.querySelector('.n-packets').value | 0;
            const selects = commandDiv.querySelectorAll('.sensor-select');
            let sensorPacketIds = [];
            for (const select of selects) {
                sensorPacketIds.push(select.value | 0);
            }
            sendCommand(CommandCode.QUERY_LIST, nPackets, ...sensorPacketIds);
        }
    },
    pause_resume_stream: { opcode: 150, dataBytes: 1,
        send: (commandDiv) => {
            const pauseResumeSelect = commandDiv.querySelector('select');
            const pauseResume = pauseResumeSelect.value;
            toggleRoombaDataPause(pauseResume === '0');
        }
    },
    scheduling_leds: { opcode: 162, dataBytes: 2,
        send: (commandDiv) => {
            const selects = commandDiv.querySelectorAll('select');
            const weekdaySelect = selects[0];
            const schedulingSelect = selects[1];
            let weekdayBits = 0;
            let schedulingBits = 0;
            for (const option of weekdaySelect.selectedOptions) {
                weekdayBits |= 1 << parseInt(option.value);
            }
            for (const option of schedulingSelect.selectedOptions) {
                schedulingBits |= 1 << parseInt(option.value);
            }
            sendCommand(CommandCode.SCHEDULING_LEDS, weekdayBits, schedulingBits);
        }
    },
    digit_leds_raw: { opcode: 163, dataBytes: 4,
        send: (commandDiv) => {
            const digits = commandDiv.querySelectorAll('select');
            let digitBytes = [];
            for (const digit of digits) {
                let digitByte = 0;
                for (const option of digit.selectedOptions) {
                    digitByte |= 1 << parseInt(option.value);
                }
                digitBytes.push(digitByte);
            }
            sendCommand(CommandCode.DIGIT_LEDS_RAW, ...digitBytes);
        }
    },
    digit_leds_ascii: { opcode: 164, dataBytes: 4,
        send: (commandDiv) => {
            const digits = commandDiv.querySelectorAll('input');
            let digitBytes = [];
            for (const digit of digits) {
                digitBytes.push(digit.value.charCodeAt(0) | 0);
            }
            sendCommand(CommandCode.DIGIT_LEDS_ASCII, ...digitBytes);
        }
    },
    buttons: { opcode: 165, dataBytes: 1,
        send: (commandDiv) => {
            const buttonSelect = commandDiv.querySelector('select');
            let buttons = 0;
            for (const option of buttonSelect.selectedOptions) {
                buttons |= 1 << parseInt(option.value);
            }
            sendCommand(CommandCode.BUTTONS, buttons);
        }
    },
    schedule: { opcode: 167, dataBytes: 15,
        send: (commandDiv) => {
            const daysSelect = commandDiv.querySelector('select');
            const times = commandDiv.querySelectorAll('input');
            let days = 0;
            for (const option of daysSelect.selectedOptions) {
                days |= 1 << parseInt(option.value);
            }
            let timesArray = [];
            for (const time of times) {
                const timeString = time.value.split(':');
                timesArray.push(parseInt(timeString[0]));
                timesArray.push(parseInt(timeString[1]));
            }
            sendCommand(CommandCode.SCHEDULE, days, ...timesArray);
        }
    },
    set_day_time: { opcode: 168, dataBytes: 3,
        send: (commandDiv) => {
            const daySelect = commandDiv.querySelector('select');
            const time = commandDiv.querySelector('input').value.split(':');
            sendCommand(CommandCode.SET_DAY_TIME, parseInt(daySelect.value), parseInt(time[0]), parseInt(time[1]));
        }
    },
    stop: { opcode: 173, dataBytes: 0 },
    wakeup: { opcode: 200, dataBytes: 0 },
    stream_song: { opcode: 201, dataBytes: -2,
        send: async (commandDiv) => {
            const fileInput = commandDiv.querySelector('#midiFile');
            const file = fileInput.files[0];
            if (!file) {
                console.error("No MIDI file selected");
                return;
            }

            const arrayBuffer = await file.arrayBuffer();
            const midiData = parseMidi(arrayBuffer);

            // Assuming parseMidi returns an array of notes and durations
            // Each note is an object with { note: <note_value>, duration: <duration_value> }
            
            // const maxSongLength = 64;
            
            // let songData = [];
            // let songLength = 0;
            // for (const note of midiData) {
            //     if (songLength >= maxSongLength) {
            //         break;
            //     }
            //     songData.push(note.note);
            //     songData.push(note.duration);
            //     songLength++;
            // }

            // sendCommand(CommandCode.SONG, 0, songLength, ...songData);

            let songNumber = 0;
            let songLength = 0;
            let songData = [];

            for (let i = 0; i < midiData.length; i++) {
                songData.push(midiData[i].note);
                songData.push(midiData[i].duration);
                songLength++;

                if (songLength === 511 || i === midiData.length - 1) {
                    //sendCommand(CommandCode.STREAM_SONG, 0, songLength, ...songData);
                    setTimeout((songLength, songData) => {
                        sendCommand(CommandCode.STREAM_SONG, songLength, ...songData);
                        // sendCommand(CommandCode.PLAY, 0);
                    }, songNumber * 300, songLength, songData);
                    songNumber++;
                    songLength = 0;
                    songData = [];
                }
            }
        }
    }
};

export function getCommandOpcode(commandCode) {
    return CommandInfo[commandCode].opcode;
}

export function getCommandDataBytes(commandCode) {
    let dataBytes = CommandInfo[commandCode].dataBytes;
    if (dataBytes === -1) {
        return "N+1";
    } else if (dataBytes === -2) {
        return "2N+2";
    }
    return dataBytes;
}

export function getCommandSendFunction(commandCode) {
    const commandInfo = CommandInfo[commandCode];
    if (commandInfo.send) {
        return commandInfo.send;
    }
    return () => {
        sendCommand(commandCode);
    };
}

export const musicNotes = {
    G_49_0: 31,
    GSHARP_51_9: 32,
    A_55_0: 33,
    ASHARP_58_3: 34,
    B_61_7: 35,
    C_65_4: 36,
    CSHARP_69_3: 37,
    D_73_4: 38,
    DSHARP_77_8: 39,
    E_82_4: 40,
    F_87_3: 41,
    FSHARP_92_5: 42,
    G_98_0: 43,
    GSHARP_103_8: 44,
    A_110_0: 45,
    ASHARP_116_5: 46,
    B_123_5: 47,
    C_130_8: 48,
    CSHARP_138_6: 49,
    D_146_8: 50,
    DSHARP_155_6: 51,
    E_164_8: 52,
    F_174_6: 53,
    FSHARP_185_0: 54,
    G_196_0: 55,
    GSHARP_207_7: 56,
    A_220_0: 57,
    ASHARP_233_1: 58,
    B_246_9: 59,
    C_261_6: 60,
    CSHARP_277_2: 61,
    D_293_7: 62,
    DSHARP_311_1: 63,
    E_329_6: 64,
    F_349_2: 65,
    FSHARP_370_0: 66,
    G_392_0: 67,
    GSHARP_415_3: 68,
    A_440_0: 69,
    ASHARP_466_2: 70,
    B_493_9: 71,
    C_523_3: 72,
    CSHARP_554_4: 73,
    D_587_3: 74,
    DSHARP_622_3: 75,
    E_659_3: 76,
    F_698_5: 77,
    FSHARP_740_0: 78,
    G_784_0: 79,
    GSHARP_830_6: 80,
    A_880_0: 81,
    ASHARP_932_4: 82,
    B_987_8: 83,
    C_1046_5: 84,
    CSHARP_1108_8: 85,
    D_1174_7: 86,
    DSHARP_1244_5: 87,
    E_1318_5: 88,
    F_1396_9: 89,
    FSHARP_1480_0: 90,
    G_1568_0: 91,
    GSHARP_1661_3: 92,
    A_1760_0: 93,
    ASHARP_1864_7: 94,
    B_1975_6: 95,
    C_2093_1: 96,
    CSHARP_2217_5: 97,
    D_2349_4: 98,
    DSHARP_2489_1: 99,
    E_2637_1: 100,
    F_2793_9: 101,
    FSHARP_2960_0: 102,
    G_3136_0: 103,
    GSHARP_3322_5: 104,
    A_3520_1: 105,
    ASHARP_3729_4: 106,
    B_3951_2: 107
};

export const defaultStreamPackets = [
    SensorPacketId.BUMPS_AND_WHEEL_DROPS,
    SensorPacketId.DISTANCE,
    SensorPacketId.ANGLE,
    SensorPacketId.CHARGING_STATE,
    SensorPacketId.CURRENT,
    SensorPacketId.TEMPERATURE,
    SensorPacketId.BATTERY_CHARGE,
    SensorPacketId.BATTERY_CAPACITY,
    SensorPacketId.OI_MODE,
    SensorPacketId.SONG_NUMBER,
    SensorPacketId.SONG_PLAYING,
    SensorPacketId.LIGHT_BUMPER,
    SensorPacketId.LEFT_MOTOR_CURRENT,
    SensorPacketId.RIGHT_MOTOR_CURRENT,
    SensorPacketId.MAIN_BRUSH_MOTOR_CURRENT,
    SensorPacketId.SIDE_BRUSH_MOTOR_CURRENT
];

export function sendCommand(commandCode, ...args) {
    const command = `${commandCode} ${args.join(' ')}`;
    sendWSCommand(command);
}

export function startRoomba() {
    sendCommand(CommandCode.WAKEUP);
    setTimeout(() => {
        sendCommand(CommandCode.START);
    }, 500);
    setTimeout(() => {
        sendCommand(CommandCode.FULL);
    }, 1000);
    if (!state.paused && !state.roombaTurnedOn) {
        setTimeout(() => {
            stream(...defaultStreamPackets);
            toggleRoombaDataPause(false);
        }, 1500);
    } else {
        afterRoombaTurnedOn();
    }
}

export function shutdownRoomba() {
    sendCommand(CommandCode.START);
    setTimeout(() => {
        sendCommand(CommandCode.STOP);
    }, 500);
    setTimeout(() => {
        sendCommand(CommandCode.POWER);
    }, 500);
    setTimeout(() => {
        afterRoombaShutdown();
    }, 1000);
}

export function stream(...sensorPacketIds) {
    sendCommand(CommandCode.STREAM, sensorPacketIds.length, ...sensorPacketIds);
}