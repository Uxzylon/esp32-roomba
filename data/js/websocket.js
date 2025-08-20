
import { afterConnect, afterDisconnect, afterRoombaTurnedOn, state, getBasePath } from "./main.js";
import { formatSensorValue } from "./sensors.js";
export let ws;
export let ipAddress;
export let port;

const roombaDataStreamElements = document.getElementsByClassName("roombaData-stream");
const roombaDataSensorsElements = document.getElementsByClassName("roombaData-sensors");
const roombaDataQueryElements = document.getElementsByClassName("roombaData-query_list");

export function connectWebSocket() {
    const currentHost = window.location.hostname;
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    
    const wsPath = `${wsProtocol}//${getBasePath(currentHost)}/ws`;
    
    console.log(`Connecting to WebSocket at: ${wsPath}`);
    ws = new WebSocket(wsPath);

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
    const needUpdate = (!state.roombaTurnedOn && !state.paused) || (state.roombaTurnedOn && !state.paused);

    if (needUpdate && (jsonData.data.oi_mode === 2 || jsonData.data.oi_mode === 3)) {
        afterRoombaTurnedOn();
    }

    const formattedData = {};
    for (const [sensorName, rawValue] of Object.entries(jsonData.data)) {
        formattedData[sensorName] = formatSensorValue(sensorName, rawValue);
    }

    let formattedText = '';
    for (const [sensorName, formattedValue] of Object.entries(formattedData)) {
        formattedText += `"${sensorName}": ${formattedValue}\n`;
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
        elements[i].textContent = formattedText;
    }
}