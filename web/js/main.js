import { connectWebSocket, disconnectWebSocket } from './websocket.js';
import { leftJoystickState, rightJoystickState, addJoystickListeners, setJoystickElements, enableJoysticks } from './joystick.js';
import { updateGamepad, setGamepadActive } from './gamepad.js';
import { sendCommand, startRoomba, shutdownRoomba, stream, defaultStreamPackets, CommandCode, getCommandOpcode, getCommandDataBytes, getCommandSendFunction, musicNotes } from './commands.js';
import { SensorPacketId } from './sensors.js';

export const state = {
    paused: false,
    roombaTurnedOn: false
};

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

    const commandSelect = document.getElementById('commandSelect');
    for (const command in CommandCode) {
        const option = document.createElement('option');
        option.value = CommandCode[command];
        option.text = command;
        commandSelect.appendChild(option);
    };

    updateMenuCommands();
    fillNoteSelections();
    fillSensorSelections();
    nPacketsSelects();
});

const connectButton = document.getElementById('connectButton');
const disconnectButton = document.getElementById('disconnectButton');
const startButton = document.getElementById('startButton');
const shutdownButton = document.getElementById('shutdownButton');
const pauseButton = document.getElementById('pauseButton');

const ipAddressInput = document.getElementById('ipAddress');
const portInput = document.getElementById('port');

const buttons = [
    document.getElementById('aButton'),
    // document.getElementById('bButton'),
    // document.getElementById('cButton'),
    // document.getElementById('dButton'),
    // document.getElementById('eButton'),
    // document.getElementById('fButton'),
    // document.getElementById('gButton')
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

    if (!state.paused) {
        stream(...defaultStreamPackets);
        toggleRoombaDataPause(false);
    } else if (!state.roombaTurnedOn && !state.paused || (state.roombaTurnedOn && state.paused)) {
        afterRoombaTurnedOn(false);
    }
}

export function afterDisconnect() {
    afterRoombaShutdown(false);

    ipAddressInput.classList.remove('connected');
    ipAddressInput.classList.add('disconnected');
    portInput.classList.remove('connected');
    portInput.classList.add('disconnected');
    
    connectButton.disabled = false;
    disconnectButton.disabled = true;

    startButton.disabled = true;
    shutdownButton.disabled = true;
}

export function afterRoombaTurnedOn(updateState = true) {
    if (updateState) {
        state.roombaTurnedOn = true;
    }

    startButton.disabled = true;
    shutdownButton.disabled = false;
    pauseButton.classList.remove('disabled');

    enableJoysticks(true);
    buttons.forEach(button => button.classList.remove('disabled'));
}

export function afterRoombaShutdown(updateState = true) {
    if (updateState) {
        state.roombaTurnedOn = false;
    }

    startButton.disabled = false;
    shutdownButton.disabled = true;
    pauseButton.classList.add('disabled');

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

export function toggleRoombaDataPause(pause) {
    if (pause === undefined) {
        pause = !state.paused;
    }

    if (pause) {
        pauseButton.innerHTML = '▶️';
        sendCommand(CommandCode.PAUSE_RESUME_STREAM, 0);
        state.paused = true;
    } else {
        pauseButton.innerHTML = '⏸';
        sendCommand(CommandCode.PAUSE_RESUME_STREAM, 1);
        state.paused = false;
    }
}

function toggleMenu() {
    const menuOverlay = document.getElementById('menuOverlay');
    menuOverlay.style.display = menuOverlay.style.display === 'flex' ? 'none' : 'flex';
}

function sendCustomCommand() {
    const commandSelect = document.getElementById('commandSelect');
    const paramsInput = document.getElementById('customCommandParams');
    sendCommand(commandSelect.value, paramsInput.value);
}

function updateMenuCommands() {
    for (const command in CommandCode) {
        const commandDiv = document.getElementById(`command-${command.toLowerCase()}`);
        if (commandDiv) {
            const divCommandRow = commandDiv.querySelector('.command-row');

            const divCommandTitle = document.createElement('div');
            divCommandTitle.classList.add('command-title');
            divCommandTitle.innerText = command;
            divCommandRow.appendChild(divCommandTitle);

            const divCommandOpcode = document.createElement('div');
            divCommandOpcode.classList.add('command-opcode');
            divCommandOpcode.innerText = `Opcode: ${getCommandOpcode(CommandCode[command])}`;
            divCommandRow.appendChild(divCommandOpcode);

            const divCommandBytes = document.createElement('div');
            divCommandBytes.classList.add('command-bytes');
            divCommandBytes.innerText = `Data Bytes: ${getCommandDataBytes(CommandCode[command])}`;
            divCommandRow.appendChild(divCommandBytes);

            const button = document.createElement('button');
            button.innerText = 'Send';
            button.onclick = () => {
                getCommandSendFunction(CommandCode[command])(commandDiv);
            }
            commandDiv.appendChild(button);
        }
    }
}

function fillNoteSelections() {
    const noteSelects = document.querySelectorAll('select.note');
    noteSelects.forEach(select => {
        let option = document.createElement('option');
        option.value = 0;
        option.textContent = '';
        select.appendChild(option);
        for (const musicNote of Object.entries(musicNotes)) {
            option = document.createElement('option');
            option.value = musicNote[1];
            const [note, frequency, subFrequency] = musicNote[0].split('_');
            option.textContent = note.replace(/_/g, ' ').replace(/SHARP/g, '#') + ' (' + frequency + '.' + subFrequency + ' Hz)';
            select.appendChild(option);
        }
    });
}

function fillSensorSelections() {
    const sensorSelects = document.querySelectorAll('select.sensor-select');
    sensorSelects.forEach(select => {
        for (const sensor of Object.entries(SensorPacketId)) {
            const option = document.createElement('option');
            option.value = sensor[1];
            option.textContent = sensor[0];
            select.appendChild(option);
        }
    });
}

function nPacketsSelects() {
    const nPacketsInputs = document.querySelectorAll('input.n-packets');
    nPacketsInputs.forEach(input => {
        const divQueryList = input.parentElement;
        input.onchange = () => {
            const nPackets = parseInt(input.value, 10);
            const existingSelects = divQueryList.querySelectorAll('select.sensor-select');
            const currentCount = existingSelects.length;

            if (nPackets < currentCount) {
                // Remove excess select elements
                for (let i = nPackets; i < currentCount; i++) {
                    existingSelects[i].remove();
                }
            } else if (nPackets > currentCount) {
                // Add new select elements
                for (let i = currentCount; i < nPackets; i++) {
                    const select = document.createElement('select');
                    select.classList.add('sensor-select');
                    for (const sensor of Object.entries(SensorPacketId)) {
                        const option = document.createElement('option');
                        option.value = sensor[1];
                        option.textContent = sensor[0];
                        select.appendChild(option);
                    }
                    divQueryList.appendChild(select);
                }
            }
        };
    });
}

// Attach functions to the window object to make them globally accessible
window.connectWebSocket = connectWebSocket;
window.disconnectWebSocket = disconnectWebSocket;
window.sendCommand = sendCommand;
window.toggleImageMode = toggleImageMode;
window.toggleMenu = toggleMenu;
window.startRoomba = startRoomba;
window.shutdownRoomba = shutdownRoomba;
window.toggleRoombaDataPause = toggleRoombaDataPause;
window.sendCustomCommand = sendCustomCommand;
window.CommandCode = CommandCode;