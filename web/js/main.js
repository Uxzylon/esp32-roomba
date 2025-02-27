import { connectWebSocket, disconnectWebSocket, sendCommand, clearRoombaData } from './websocket.js';
import { leftJoystickState, rightJoystickState, addJoystickListeners, setJoystickElements, enableJoysticks } from './joystick.js';
import { updateGamepad, setGamepadActive } from './gamepad.js';
import { startRoomba, shutdownRoomba } from './commands.js';

document.addEventListener('DOMContentLoaded', function() {
    let savedIpAddress = localStorage.getItem('ipAddress');
    if (savedIpAddress) {
        document.getElementById('ipAddress').value = savedIpAddress; // Set saved IP address
    }
    let savedPort = localStorage.getItem('port');
    if (savedPort) {
        document.getElementById('port').value = savedPort; // Set saved port
    }

    const joystickLeft = document.getElementById('joystickLeft');
    const joystickRight = document.getElementById('joystickRight');
    const joystickLeftHandle = joystickLeft.querySelector('.joystick-handle');
    const joystickRightHandle = joystickRight.querySelector('.joystick-handle');

    setJoystickElements(joystickLeft, joystickRight, joystickLeftHandle, joystickRightHandle);

    addJoystickListeners(joystickLeft, joystickLeftHandle, function(x, y) {
        leftJoystickState.y = y;
    }, function() {
        leftJoystickState.y = 0;
    }, false, true, leftJoystickState); // Restrict X movement

    addJoystickListeners(joystickRight, joystickRightHandle, function(x, y) {
        rightJoystickState.x = x;
    }, function() {
        rightJoystickState.x = 0;
    }, true, false, rightJoystickState); // Restrict Y movement

    window.addEventListener("gamepadconnected", function(e) {
        console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
            e.gamepad.index, e.gamepad.id,
            e.gamepad.buttons.length, e.gamepad.axes.length);
        setGamepadActive(true);
        requestAnimationFrame(updateGamepad);
    });

    window.addEventListener("gamepaddisconnected", function(e) {
        console.log("Gamepad disconnected from index %d: %s",
            e.gamepad.index, e.gamepad.id);
        setGamepadActive(false);
    });

    const backgroundImage = document.getElementById('backgroundImage');
    backgroundImage.style.objectFit = 'contain';
});

const connectButton = document.getElementById('connectButton');
const disconnectButton = document.getElementById('disconnectButton');
const startButton = document.getElementById('startButton');
const shutdownButton = document.getElementById('shutdownButton');

const ipAddressInput = document.getElementById('ipAddress');
const portInput = document.getElementById('port');

const buttons = [
    document.getElementById('aButton'),
    document.getElementById('bButton'),
    document.getElementById('cButton'),
    document.getElementById('dButton'),
    document.getElementById('eButton'),
    document.getElementById('fButton'),
    document.getElementById('gButton')
];

export function afterConnect() {
    ipAddressInput.classList.remove('disconnected');
    ipAddressInput.classList.add('connected');
    portInput.classList.remove('disconnected');
    portInput.classList.add('connected');
    
    connectButton.disabled = true;
    disconnectButton.disabled = false;

    startButton.disabled = false;
    shutdownButton.disabled = true;
}

export function afterDisconnect() {
    afterRoombaShutdown();

    ipAddressInput.classList.remove('connected');
    ipAddressInput.classList.add('disconnected');
    portInput.classList.remove('connected');
    portInput.classList.add('disconnected');
    
    connectButton.disabled = false;
    disconnectButton.disabled = true;

    startButton.disabled = true;
    shutdownButton.disabled = true;
}

export function afterRoombaTurnedOn() {
    startButton.disabled = true;
    shutdownButton.disabled = false;

    enableJoysticks(true);
    buttons.forEach(button => button.classList.remove('disabled'));
}

export function afterRoombaShutdown() {
    clearRoombaData();
    startButton.disabled = false;
    shutdownButton.disabled = true;

    enableJoysticks(false);
    buttons.forEach(button => button.classList.add('disabled'));
}

function toggleImageMode() {
    const backgroundImage = document.getElementById('backgroundImage');
    if (backgroundImage.style.objectFit === 'contain') {
        backgroundImage.style.objectFit = 'cover';
    } else {
        backgroundImage.style.objectFit = 'contain';
    }
}

// Attach functions to the window object to make them globally accessible
window.connectWebSocket = connectWebSocket;
window.disconnectWebSocket = disconnectWebSocket;
window.sendCommand = sendCommand;
window.toggleImageMode = toggleImageMode;
window.startRoomba = startRoomba;
window.shutdownRoomba = shutdownRoomba;