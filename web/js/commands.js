import { sendCommand, clearRoombaData } from './websocket.js';
import { disableButton, enableButton } from './utils.js';

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
        clearRoombaData();
        enableButton('startButton');
        disableButton('shutdownButton');
    }, 1000);
}