
import { afterConnect, afterDisconnect, afterRoombaTurnedOn, state, getHostname } from "./main.js";
import { formatSensorValue } from "./sensors.js";
export let ws;
export let ipAddress;
export let port;

const roombaDataStreamElements = document.getElementsByClassName("roombaData-stream");
const roombaDataSensorsElements = document.getElementsByClassName("roombaData-sensors");
const roombaDataQueryElements = document.getElementsByClassName("roombaData-query_list");

let connectionAttemptInProgress = false;

export function connectWebSocket() {
    if (connectionAttemptInProgress) return;
    connectionAttemptInProgress = true;

    const currentHost = getHostname(window.location.hostname);
    const currentPath = window.location.pathname;
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';

    if (currentHost !== window.location.hostname) {
        initWebSocketConnection(`${wsProtocol}//${currentHost}`);
        connectionAttemptInProgress = false;
    } else {
        // Build the healthcheck URL based on current host and path
        let healthcheckUrl;
        if (currentPath === "/" || currentPath === "") {
            healthcheckUrl = `${window.location.origin}/healthcheck`;
        } else {
            // Remove trailing slash if present
            const basePath = currentPath.endsWith("/") ? currentPath.slice(0, -1) : currentPath;
            healthcheckUrl = `${window.location.origin}${basePath}/healthcheck`;
        }

        fetch(healthcheckUrl)
            .then(response => {
                const isDirectConnection = response.headers.has('X-ESP32');

                let wsPath;
                if (isDirectConnection) {
                    const baseHost = currentHost.split(':')[0]; // Remove any port
                    wsPath = `${wsProtocol}//${baseHost}:81`;
                } else {
                    let wsBasePath = currentHost;
                    const pathPrefix = currentPath.endsWith('/') ? 
                        currentPath.slice(0, -1) : 
                        currentPath.substring(0, currentPath.lastIndexOf('/'));
                    if (pathPrefix && !pathPrefix.includes('.')) {
                        wsPath = `${wsProtocol}//${wsBasePath}${pathPrefix}/ws`;
                    } else {
                        wsPath = `${wsProtocol}//${wsBasePath}/ws`;
                    }
                }
                initWebSocketConnection(wsPath);
            })
            .catch(error => {
                console.error("Error checking for direct connection:", error);
                // Fallback to direct connection on port 81 if the check fails
                const baseHost = currentHost.split(':')[0]; // Remove any port
                const wsPath = `${wsProtocol}//${baseHost}:81`;
                console.log(`Connection check failed - falling back to: ${wsPath}`);
                initWebSocketConnection(wsPath);
            })
            .finally(() => {
                connectionAttemptInProgress = false;
            });
    }
}

function initWebSocketConnection(wsPath) {
    console.log(`Connecting to WebSocket at: ${wsPath}`);
    ws = new WebSocket(wsPath);

    ws.onopen = function() {
        console.log('WebSocket connection opened');
        afterConnect();
    };

    ws.onmessage = function(event) {
        try {
            const jsonData = JSON.parse(event.data);
            displayRoombaData(event.data);
        } catch (e) {
            console.error("Error parsing WebSocket message:", e);
        }
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

    if (
        needUpdate &&
        jsonData.data &&
        typeof jsonData.data.oi_mode !== "undefined" &&
        (jsonData.data.oi_mode === 2 || jsonData.data.oi_mode === 3)
    ) {
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
