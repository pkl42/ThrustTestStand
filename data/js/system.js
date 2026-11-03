// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

import { $ } from './helpers.js';
import {bindElements} from './binder.js';
import {setEscDriverEnabled} from './calibration.js';

window.latestMotorMount = 0; // 0 = disarmed, 2 = armed, etc.

	const SensorStateMap = {
		0: { text: "Uninitialized", css: "state-uninit" },
		1: { text: "Initializing",  css: "state-init" },
		2: { text: "Ready",         css: "state-ready" },
		3: { text: "Error",         css: "state-error" },
		4: { text: "Disabled",      css: "state-disabled" }
	};
	
export function init() {
    if (init.done) return;
    init.done = true;
	
	subscribeSystemSettings();
//    subscribeLiveUpdate();
	bindUI();
}

export function setPassthroughBanner(active) {
		const banner = document.getElementById("passthroughBanner");
		banner.classList.toggle("active", active);
	}	
	
export function updateStateElement(id, stateValue) {
		const el = document.getElementById(id);
		if (!el || stateValue === undefined) return;

		const map = SensorStateMap[stateValue] ||
					{ text: "Unknown", css: "state-uninit" };

		el.textContent = map.text;
		el.className = "state " + map.css;
	}
	
	const MotorMountStateMap = {
		0: { text: "Disarmed", css: "state-uninit" },
		1: { text: "Arming",  css: "state-init" },
		2: { text: "Armed",  css: "state-ready" },
		3: { text: "Running", css: "state-ready" }
	};
	

	
export function toggleMotor({ el }) {
	 const btn = el;
    const mountState = window.latestMotorMount ;

    let url = "";
    if (mountState === 0) {
        url = "/api/system/arm";
    } else if (mountState === 2) {
        url = "/api/system/disarm";
    } else {
        console.warn("Motor toggle not allowed in this state:", mountState);
        return;
    }

    btn.disabled = true; // prevent multiple clicks
    fetch(url, { method: "POST" })
        .then(r => {
            if (!r.ok) throw new Error("Request failed: " + r.status);
            return r.json();
        })
        .then(resp => {
            console.log("Motor toggle successful:", resp);
            //updateSystemState();
        })
        .catch(err => {
            console.error("Motor toggle failed:", err);
            btn.disabled = false; // re-enable on error
        });
}



export function updateMountStateElement(id, stateValue) {
		const el = document.getElementById(id);
		if (!el || stateValue === undefined) return;

		const map = MotorMountStateMap[stateValue] ||
					{ text: "Unknown", css: "state-uninit" };

		el.textContent = map.text;
		el.className = "state " + map.css;
	}
	
	const ActuatorStateMap = {
		0: { text: "Uninitialized", css: "state-uninit" },
		1: { text: "Initializing",  css: "state-init" },
		2: { text: "Ready",         css: "state-ready" },
		3: { text: "Active",        css: "state-ready" },
		4: { text: "Stopped",       css: "state-ready" },
		5: { text: "Error",         css: "state-error" },
		6: { text: "Disabled",      css: "state-disabled" },
		7: { text: "E-Stop",      css: "state-error" }
	};	
	
export function updateActuatorElement(id, stateValue) {
		const el = document.getElementById(id);
		if (!el || stateValue === undefined) return;

		const map = ActuatorStateMap[stateValue] ||
					{ text: "Unknown", css: "state-uninit" };

		el.textContent = map.text;
		el.className = "state " + map.css;
	}
	
	
	
function subscribeSystemSettings() {
    document.addEventListener('ws:data', (e) => {
        const s = e.detail;

        if (s.type !== 'system') return;

        try {
            // ── Sensor / state indicators
            updateStateElement("state-thrust", s.thrust);
            updateStateElement("state-torque", s.torque);
            updateStateElement("state-thermo", s.temperature);
            updateStateElement("state-rpm", s.rpm);
            updateStateElement("state-current", s.current);
            updateStateElement("state-voltage", s.voltage);

            // ── Actuator states
            updateActuatorElement("state-motor", s.motor);
            updateMountStateElement("state-motor-mount", s.motor_mount);

            // ── Sync UI state
            if (s.motor_mount !== undefined) {
                window.latestMotorMount = s.motor_mount;
                updateMotorButton(s.motor_mount);
				setEscDriverEnabled(window.latestMotorMount===0);
			}

            // ── Mode dropdown sync
            const modeSelect = document.getElementById("system.mode");
            if (modeSelect && s.mode && modeSelect.value !== s.mode) {
                modeSelect.value = s.mode;
            }

            // ── Passthrough banner
            const isPassthrough = s.mode === "MODE_ESC_PASSTHROUGH";
            setPassthroughBanner(isPassthrough);

        } catch (err) {
            console.warn("System WS update failed:", err);
        }
    });
}


function bindUI() {
	    bindElements([
        {
            id: "system_armMotor",
            event: "click",
            handler: ({ el }) => {
                toggleMotor({ el });
            }
		}
    ]);
}

function updateMotorButton(mountState) {
	const btn = document.getElementById("system_armMotor");
	if (!btn) return;

	switch (mountState) {
		case 0: // Disarmed
			btn.textContent = "Arm Motor";
			btn.disabled = false;
			break;
		case 2: // Armed
			btn.textContent = "Disarm Motor";
			btn.disabled = false;
			break;
		default: // other states
			btn.textContent = "Arm/Disarm Motor";
			btn.disabled = true;
			break;
	}
}