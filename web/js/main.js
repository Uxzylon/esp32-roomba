import { connectWebSocket, disconnectWebSocket, sendCommand } from './websocket.js';
import { leftJoystickState, rightJoystickState, addJoystickListeners, setJoystickElements } from './joystick.js';
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