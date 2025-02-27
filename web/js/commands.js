import { sendCommand } from './websocket.js';
import { afterRoombaShutdown } from './main.js';

export function startRoomba() {
    sendCommand('wakeup');
    setTimeout(() => {
        sendCommand('full');
    }, 500);
}

export function shutdownRoomba() {
    sendCommand('start');
    setTimeout(() => {
        sendCommand('power');
    }, 500);
    setTimeout(() => {
        afterRoombaShutdown();
    }, 1000);
}