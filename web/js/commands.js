import { sendWSCommand } from './websocket.js';
import { afterRoombaShutdown } from './main.js';
import { SensorPacketId } from './sensors.js';
import { state } from './main.js';

export const CommandCode = {
    WAKEUP: "wakeup",
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
    STOP: "stop"
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
    if (state.paused) {
        setTimeout(() => {
            toggleRoombaDataPause();
        }, 1500);
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