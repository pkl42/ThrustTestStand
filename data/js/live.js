// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

import { $ } from './helpers.js';
import {bindElements} from './binder.js';
import { isExecutable } from './safety.js';

export function init() {
    if (init.done) return;
    init.done = true;
	
	subscribeLive();
	bindUI();
}

export function resetThrottleUI() {
    $('throttle').value = 0;
    $('throttleVal').innerText = "0";
}

export function setThrottle(val) {
	document.getElementById('throttleVal').innerText = val;

	fetch('/api/live/setThrottle', {
		method: 'POST',
		headers: {
			'Content-Type': 'application/x-www-form-urlencoded'
		},
		body: 'throttle=' + encodeURIComponent(val)
	});
}

export function tare(event) {
	const btn = event.target;
	btn.disabled = true;

	fetch('/api/live/tare', { method: 'POST' })
		.then(response => {
			if (!response.ok) {
				throw new Error('Tare failed');
			}
			return response.text();
		})
		.then(text => {
			console.log('Tare response:', text);
			alert('Sensors tared successfully');
		})
		.catch(err => {
			console.error(err);
			alert('Error taring sensors');
		})
		.finally(() => {
			btn.disabled = false;
		});
}

let throttleTimer; 

function onThrottleChange(el) {
    // el is the <input type="range"> element
    const value = el.value;
    const display = el.nextElementSibling; // assumes <span> follows the slider

    // cancel previous scheduled call
    clearTimeout(throttleTimer);

    throttleTimer = setTimeout(() => {
        if (!isExecutable()) {
            alert(
                "Motor disabled: safety trip active or unarmed or unsaved safety/configuration settings!"
            );
            // reset slider to safe position
            el.value = 0;
            display.textContent = "0";
            return;
        }

        // update UI
        display.textContent = parseFloat(value).toFixed(1);

        // call backend / motor control
        setThrottle(parseFloat(value));
    }, 50);
}
	


//
// the live state is pushed from backend continues, 
// therefor this event is dispatched for other modules usage
function subscribeLive() {
    document.addEventListener('ws:data', (e) => {
        const data = e.detail;

        if (data.type !== 'live') return;

        try {
            // ── Core telemetry
            $('thrust').innerText = data.thrust?.toFixed(2) ?? '-';
            $('torque').innerText = data.torque?.toFixed(2) ?? '-';
            $('torqueCell1').innerText = data.torqueCell1?.toFixed(2) ?? '-';
            $('torqueCell2').innerText = data.torqueCell2?.toFixed(2) ?? '-';

            $('voltage').innerText = data.voltage?.toFixed(2) ?? '-';
            $('current').innerText = data.current?.toFixed(2) ?? '-';
            $('power').innerText = data.power?.toFixed(2) ?? '-';

            $('thrust_ratio').innerText = data.thrust_ratio?.toFixed(2) ?? '-';
            $('rpm').innerText = data.rpm?.toFixed(0) ?? '-';
            $('temperature').innerText = data.temperature?.toFixed(2) ?? '-';

            // ── Throttle sync
            if (data.throttle_cmd !== undefined) {
                $('throttleVal').innerText = data.throttle_cmd.toFixed(1);
                $('throttle').value = data.throttle_cmd.toFixed(1);
            }
			
			document.dispatchEvent(
            new CustomEvent('live:update', { detail: data }));

        } catch (err) {
            console.warn("Live update error:", err);
        }
    });
}


function bindUI() {
    bindElements([
        {
            id: "throttle",
            event: "input",
            handler: ({ el }) => onThrottleChange(el)
        }
    ]);
}

