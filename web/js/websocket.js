
import { afterConnect, afterDisconnect, afterRoombaTurnedOn } from "./main.js";
export let ws;
export let ipAddress;
export let port;

const roombaDataElement = document.getElementById('roombaData');

export function connectWebSocket() {
    ipAddress = document.getElementById('ipAddress').value;
    localStorage.setItem('ipAddress', ipAddress);
    port = document.getElementById('port').value || 81;
    if (port != 81) {
        localStorage.setItem('port', port);
    }
    ws = new WebSocket('ws://' + ipAddress + ':' + port);

    ws.onopen = function() {
        console.log('WebSocket connection opened');
        afterConnect();
    };

    ws.onmessage = function(event) {
        displayRoombaData(event.data);
    };

    ws.onclose = function() {
        console.log('WebSocket connection closed');
        afterDisconnect();
    };
}

export function clearRoombaData() {
    roombaDataElement.textContent = '';
}

export function disconnectWebSocket() {
    if (ws) {
        ws.close();
    }
}

export function sendWSCommand(command) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(command);
        console.log('Sent command: ' + command);
    }
}

function displayRoombaData(data) {
    let jsonData = JSON.parse(data);
    let text = JSON.stringify(jsonData, null, 2);
    text = text.substring(1, text.length - 1);
    text = text.replace(/  /g, '');
    text = text.substring(text.indexOf('\n') + 1);
    if (roombaDataElement.textContent === '' && text) {
        afterRoombaTurnedOn();
    }
    roombaDataElement.textContent = text;
}