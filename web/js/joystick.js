export let leftJoystickState = { x: 0, y: 0 };
export let rightJoystickState = { x: 0, y: 0 };
export let joystickLeft, joystickRight, joystickLeftHandle, joystickRightHandle;

export function handleJoystickMove(event, joystick, handle, callback, restrictX, restrictY, joystickState) {
    let rect = joystick.getBoundingClientRect();
    let x = (event.touches ? event.touches[0].clientX : event.clientX) - rect.left;
    let y = (event.touches ? event.touches[0].clientY : event.clientY) - rect.top;
    let centerX = rect.width / 2;
    let centerY = rect.height / 2;
    let dx = x - centerX;
    let dy = y - centerY;
    let distance = Math.sqrt(dx * dx + dy * dy);
    let maxDistance = restrictX ? rect.width / 2 : rect.height / 2;

    if (distance > maxDistance) {
        dx = dx * maxDistance / distance;
        dy = dy * maxDistance / distance;
    }

    if (restrictX) dx = Math.min(Math.max(dx, -maxDistance), maxDistance);
    if (restrictY) dy = Math.min(Math.max(dy, -maxDistance), maxDistance);

    if (restrictX) dy = 0; // Ensure no vertical movement for horizontal joystick
    if (restrictY) dx = 0; // Ensure no horizontal movement for vertical joystick

    handle.style.left = (centerX + dx) + 'px';
    handle.style.top = (centerY + dy) + 'px';

    joystickState.x = dx / maxDistance;
    joystickState.y = dy / maxDistance;

    callback(dx / maxDistance, dy / maxDistance);
    updateDriveCommand();
}

export function handleJoystickEnd(handle, joystickState) {
    handle.style.left = '50%';
    handle.style.top = '50%';
    joystickState.x = 0;
    joystickState.y = 0;
    updateDriveCommand();
}

export function updateJoystickHandle(joystick, handle, x, y, restrictX, restrictY) {
    let rect = joystick.getBoundingClientRect();
    let centerX = rect.width / 2;
    let centerY = rect.height / 2;
    let maxDistance = rect.width / 2;

    let dx = x * maxDistance;
    let dy = y * maxDistance;

    if (restrictX) dx = 0;
    if (restrictY) dy = 0;

    handle.style.left = (centerX + dx) + 'px';
    handle.style.top = (centerY + dy) + 'px';
}

export function addJoystickListeners(joystick, handle, moveCallback, endCallback, restrictX, restrictY, joystickState) {
    let activeTouchId = null;

    joystick.addEventListener('touchstart', function(event) {
        if (activeTouchId === null) {
            activeTouchId = event.changedTouches[0].identifier;
            handleJoystickMove(event.changedTouches[0], joystick, handle, moveCallback, restrictX, restrictY, joystickState);
            event.preventDefault();
        }
    });

    joystick.addEventListener('touchmove', function(event) {
        for (let i = 0; i < event.changedTouches.length; i++) {
            if (event.changedTouches[i].identifier === activeTouchId) {
                handleJoystickMove(event.changedTouches[i], joystick, handle, moveCallback, restrictX, restrictY, joystickState);
                event.preventDefault();
                break;
            }
        }
    });

    joystick.addEventListener('touchend', function(event) {
        for (let i = 0; i < event.changedTouches.length; i++) {
            if (event.changedTouches[i].identifier === activeTouchId) {
                handleJoystickEnd(handle, joystickState);
                endCallback();
                activeTouchId = null;
                event.preventDefault();
                break;
            }
        }
    });

    joystick.addEventListener('touchcancel', function(event) {
        for (let i = 0; i < event.changedTouches.length; i++) {
            if (event.changedTouches[i].identifier === activeTouchId) {
                handleJoystickEnd(handle, joystickState);
                endCallback();
                activeTouchId = null;
                event.preventDefault();
                break;
            }
        }
    });

    joystick.addEventListener('mousedown', function(event) {
        handleJoystickMove(event, joystick, handle, moveCallback, restrictX, restrictY, joystickState);
        document.addEventListener('mousemove', mouseMoveHandler);
        document.addEventListener('mouseup', mouseUpHandler);
        event.preventDefault();
    });

    function mouseMoveHandler(event) {
        handleJoystickMove(event, joystick, handle, moveCallback, restrictX, restrictY, joystickState);
        event.preventDefault();
    }

    function mouseUpHandler(event) {
        handleJoystickEnd(handle, joystickState);
        endCallback();
        document.removeEventListener('mousemove', mouseMoveHandler);
        document.removeEventListener('mouseup', mouseUpHandler);
        event.preventDefault();
    }
}

export function setJoystickElements(left, right, leftHandle, rightHandle) {
    joystickLeft = left;
    joystickRight = right;
    joystickLeftHandle = leftHandle;
    joystickRightHandle = rightHandle;
}

export function updateDriveCommand() {
    let leftSpeed = Math.round(leftJoystickState.y * -500 + rightJoystickState.x * 500);
    let rightSpeed = Math.round(leftJoystickState.y * -500 - rightJoystickState.x * 500);

    // Normalize the speeds to ensure they stay within the range of -500 to 500
    let maxSpeed = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (maxSpeed > 500) {
        leftSpeed = (leftSpeed / maxSpeed) * 500;
        rightSpeed = (rightSpeed / maxSpeed) * 500;
    }

    sendCommand('drive_direct ' + leftSpeed + ' ' + rightSpeed);
}

export function enableJoysticks(enabled) {
    if (enabled) {
        joystickLeft.classList.remove('disabled');
        joystickRight.classList.remove('disabled');
    } else {
        joystickLeft.classList.add('disabled');
        joystickRight.classList.add('disabled');
    }
}