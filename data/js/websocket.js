
import { afterConnect, afterDisconnect, afterRoombaTurnedOn, state, getBasePath } from "./main.js";
export let ws;
export let ipAddress;
export let port;

const roombaDataStreamElements = document.getElementsByClassName("roombaData-stream");
const roombaDataSensorsElements = document.getElementsByClassName("roombaData-sensors");
const roombaDataQueryElements = document.getElementsByClassName("roombaData-query_list");

export function connectWebSocket() {
    const currentHost = window.location.hostname;
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    
    const basePath = getBasePath();
    
    let wsPath;
    
    // Use the existence of a basePath to determine if we're behind a proxy
    if (basePath) {
        // Using reverse proxy
        wsPath = `${wsProtocol}//${currentHost}${basePath}ws`;
    } else {
        // Direct access to ESP32 using port 81 for WebSockets
        port = 81; // WebSocket port
        wsPath = `${wsProtocol}//${currentHost}:${port}`;
    }
    
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