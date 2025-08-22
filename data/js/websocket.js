
import { afterConnect, afterDisconnect, afterRoombaTurnedOn, state, getHostname } from "./main.js";
import { formatSensorValue } from "./sensors.js";
export let ws;
export let ipAddress;
export let port;

const roombaDataStreamElements = document.getElementsByClassName("roombaData-stream");
const roombaDataSensorsElements = document.getElementsByClassName("roombaData-sensors");
const roombaDataQueryElements = document.getElementsByClassName("roombaData-query_list");

let connectionAttemptInProgress = false;

let storedSensorData = {};

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
        // Handle binary messages containing combined data
        if (event.data instanceof Blob) {
            processCombinedMessage(event.data);
            return;
        }
        
        // Handle legacy text messages
        try {
            const jsonData = JSON.parse(event.data);
            displayRoombaData(event.data);
        } catch (e) {
            console.error("Error parsing WebSocket message:", e);
        }
    };

    ws.onclose = function() {
        console.log('WebSocket connection closed');
        storedSensorData = {};
        afterDisconnect();
    };
}

function processCombinedMessage(binaryData) {
    // Create a DataView to read binary data
    const blob = new Blob([binaryData]);
    
    // Convert blob to ArrayBuffer
    blob.arrayBuffer().then(buffer => {
        const dataView = new DataView(buffer);
        
        // Check magic number ("RCMB")
        const magic = String.fromCharCode(
            dataView.getUint8(0),
            dataView.getUint8(1),
            dataView.getUint8(2),
            dataView.getUint8(3)
        );
        
        if (magic !== "RCMB") {
            console.error("Invalid binary message format");
            return;
        }
        
        // Read header
        const flags = dataView.getUint8(4);
        const hasSensorData = (flags & 1) === 1;
        const hasCameraFrame = (flags & 2) === 2;
        
        // Get sizes
        const sensorDataSize = dataView.getUint32(8, true); // little endian
        const cameraFrameSize = dataView.getUint32(12, true); // little endian
        
        // Process sensor data if present
        if (hasSensorData && sensorDataSize > 0) {
            const sensorDataStart = 16; // After header
            const sensorDataEnd = sensorDataStart + sensorDataSize;
            
            // Extract sensor data as string
            const sensorDataBlob = new Blob([buffer.slice(sensorDataStart, sensorDataEnd)]);
            sensorDataBlob.text().then(jsonText => {
                try {
                    displayRoombaData(jsonText);
                } catch (e) {
                    console.error("Error parsing sensor data:", e);
                }
            });
        }
        
        // Process camera frame if present
        if (hasCameraFrame && cameraFrameSize > 0) {
            const frameDataStart = 16 + sensorDataSize;
            const frameDataEnd = frameDataStart + cameraFrameSize;
            
            // Extract camera frame
            const frameBlob = new Blob([buffer.slice(frameDataStart, frameDataEnd)], {type: 'image/jpeg'});
            const url = URL.createObjectURL(frameBlob);
            
            const img = document.getElementById('backgroundImage');
            
            // Clean up previous object URL
            if (img.src && img.src.startsWith('blob:')) {
                URL.revokeObjectURL(img.src);
            }
            
            img.src = url;
            img.style.display = 'block';
            
            // Hide the placeholder if visible
            const placeholder = document.getElementById('videoPlaceholder');
            if (placeholder) {
                placeholder.style.display = 'none';
            }
        }
    }).catch(error => {
        console.error("Error processing binary message:", error);
    });
}

function handleCameraFrame(binaryData) {
    try {
        const blob = new Blob([binaryData], {type: 'image/jpeg'});
        const url = URL.createObjectURL(blob);
        
        const img = document.getElementById('backgroundImage');
        
        // Clean up previous object URL
        if (img.src && img.src.startsWith('blob:')) {
            URL.revokeObjectURL(img.src);
        }
        
        img.src = url;
        img.style.display = 'block';
        
        // Hide the placeholder if visible
        const placeholder = document.getElementById('videoPlaceholder');
        if (placeholder) {
            placeholder.style.display = 'none';
        }
    } catch (error) {
        console.error('Error processing camera frame:', error);
    }
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

    // Check for OI_MODE to determine if Roomba is turned on
    if (
        needUpdate &&
        jsonData.data &&
        typeof jsonData.data.oi_mode !== "undefined" &&
        (jsonData.data.oi_mode === 2 || jsonData.data.oi_mode === 3)
    ) {
        afterRoombaTurnedOn();
    }

    // Update our stored sensor data
    if (jsonData.data) {
        for (const [sensorName, rawValue] of Object.entries(jsonData.data)) {
            if (rawValue === null) {
                // Remove sensors that are no longer present
                delete storedSensorData[sensorName];
            } else {
                // Update or add sensor values
                storedSensorData[sensorName] = rawValue;
            }
        }
    }

    // Format all stored sensor data
    const formattedData = {};
    for (const [sensorName, rawValue] of Object.entries(storedSensorData)) {
        formattedData[sensorName] = formatSensorValue(sensorName, rawValue);
    }

    // Build the display text
    let formattedText = '';
    for (const [sensorName, formattedValue] of Object.entries(formattedData)) {
        formattedText += `"${sensorName}": ${formattedValue}\n`;
    }

    // Update the appropriate display elements
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
