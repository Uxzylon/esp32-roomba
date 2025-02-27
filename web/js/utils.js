function enableDisableButton(buttonId, enable) {
    let button = document.getElementById(buttonId);
    button.disabled = !enable;
}

export function enableButton(buttonId) {
    enableDisableButton(buttonId, true);
}

export function disableButton(buttonId) {
    enableDisableButton(buttonId, false);
}