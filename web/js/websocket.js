
import { afterConnect, afterDisconnect, afterRoombaTurnedOn, state } from "./main.js";
export let ws;
export let ipAddress;
export let port;

const roombaDataStreamElements = document.getElementsByClassName("roombaData-stream");
const roombaDataSensorsElements = document.getElementsByClassName("roombaData-sensors");
const roombaDataQueryElements = document.getElementsByClassName("roombaData-query_list");

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
    let text = JSON.stringify(jsonData.data, null, 2);
    text = text.replace(/  /g, '');
    text = text.substring(1, text.length - 1);
    text = text.substring(text.indexOf('\n') + 1);

    if ((!state.roombaTurnedOn && !state.paused) || (state.roombaTurnedOn && !state.paused)) {
        afterRoombaTurnedOn();
    }

    let elements;
    if (jsonData.type === "stream") {
        elements = roombaDataStreamElements;
    } else if (jsonData.type === "sensors") {
        elements = roombaDataSensorsElements;
    } else if (jsonData.type === "query_list") {
        elements = roombaDataQueryElements;
    } else {
        console.error("Unknown data type: " + jsonData.type);
        return;
    }

    for (let i = 0; i < elements.length; i++) {
        elements[i].textContent = text;
    }
}