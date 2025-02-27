
import { disableButton, enableButton } from "./utils.js";
export let ws;
export let ipAddress;
export let port;

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
        document.getElementById('ipAddress').classList.remove('disconnected');
        document.getElementById('ipAddress').classList.add('connected');
        document.getElementById('port').classList.remove('disconnected');
        document.getElementById('port').classList.add('connected');
        
        let startButton = document.getElementById('startButton');
        let shutdownButton = document.getElementById('shutdownButton');
        startButton.disabled = false;
        shutdownButton.disabled = true
    };

    ws.onmessage = function(event) {
        displayRoombaData(event.data);
    };

    ws.onclose = function() {
        console.log('WebSocket connection closed');
        document.getElementById('ipAddress').classList.remove('connected');
        document.getElementById('ipAddress').classList.add('disconnected');
        document.getElementById('port').classList.remove('connected');
        document.getElementById('port').classList.add('disconnected');
        
        disableButton('startButton');
        disableButton('shutdownButton');

        clearRoombaData();
    };
}

export function clearRoombaData() {
    let roombaDataElement = document.getElementById('roombaData');
    roombaDataElement.textContent = '';
}

export function disconnectWebSocket() {
    if (ws) {
        ws.close();
        document.getElementById('ipAddress').classList.remove('connected');
        document.getElementById('ipAddress').classList.add('disconnected');
    }
}

export function sendCommand(command) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(command);
    }
}

function displayRoombaData(data) {
    let roombaDataElement = document.getElementById('roombaData');
    
    let jsonData = JSON.parse(data);
    let text = JSON.stringify(jsonData, null, 2);
    text = text.substring(1, text.length - 1);
    text = text.replace(/  /g, '');
    text = text.substring(text.indexOf('\n') + 1);
    roombaDataElement.textContent = text;

    if (text) {
        disableButton('startButton');
        enableButton('shutdownButton');
    } else {
        enableButton('startButton');
        disableButton('shutdownButton');
    }
}