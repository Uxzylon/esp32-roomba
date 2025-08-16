import { updateDriveCommand } from './joystick.js';
import { leftJoystickState, rightJoystickState, updateJoystickHandle, joystickLeft, joystickLeftHandle, joystickRight, joystickRightHandle } from './joystick.js';

let lastGamepadDriveValues = { left: 0, right: 0 };
let gamepadActive = false;
let gamepadMode = 1;
let triggersInitialized = { left: false, right: false };

export function updateGamepad() {
    let gamepads = navigator.getGamepads();
    if (gamepads[0]) {
        let gp = gamepads[0];
        let leftSpeed = 0;
        let rightSpeed = 0;

        if (gamepadMode === 1) {
            // Mode 1: Triggers for forward/reverse, left joystick for left/right
            triggersInitialized.left = triggersInitialized.left || gp.axes[4] !== 0;
            triggersInitialized.right = triggersInitialized.right || gp.axes[5] !== 0;

            let leftTriggerValue = triggersInitialized.left ? gp.axes[4] : -1;
            let rightTriggerValue = triggersInitialized.right ? gp.axes[5] : -1;

            leftSpeed = (leftTriggerValue - rightTriggerValue) * 0.5; // Left trigger - Right trigger
            rightSpeed = gp.axes[0]; // Left stick horizontal axis
        } else {
            // Mode 2: Left stick forward/backward, right stick left/right
            leftSpeed = gp.axes[1]; // Left stick vertical axis
            rightSpeed = gp.axes[2]; // Right stick horizontal axis
        }

        if (leftSpeed !== lastGamepadDriveValues.left || rightSpeed !== lastGamepadDriveValues.right) {
            leftJoystickState.y = leftSpeed;
            rightJoystickState.x = rightSpeed;
            lastGamepadDriveValues.left = leftJoystickState.y;
            lastGamepadDriveValues.right = rightJoystickState.x;
            updateDriveCommand();

            // Update the joystick handles visually
            updateJoystickHandle(joystickLeft, joystickLeftHandle, 0, leftJoystickState.y, true, false);
            updateJoystickHandle(joystickRight, joystickRightHandle, rightJoystickState.x, 0, false, true);
        }
        setGamepadActive(true);
    } else {
        setGamepadActive(false);
    }
    requestAnimationFrame(updateGamepad);
}

export function setGamepadActive(active) {
    gamepadActive = active;
}

export function isGamepadActive() {
    return gamepadActive;
}