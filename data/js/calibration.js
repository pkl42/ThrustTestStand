// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

import { setValue,populateSelect } from './helpers.js';

export function init() {
    if (init.done) return;
    init.done = true;
	
	loadEscDriverOptions();
	loadRPMOptions();
	subscribeCalibration();
}

function subscribeCalibration() {
    document.addEventListener('ws:data', (e) => {
        const cfg = e.detail;

        if (cfg.type !== 'calibration') return;

        try {
            /* ---------- Version ---------- */
            if (typeof cfg.version === "string") {
                document.title = "Motor Thrust Test Stand " + cfg.version;

                const h1 = document.getElementById('page-title');
                if (h1) {
                    h1.textContent =
                        "Motor Thrust Test Stand: " + cfg.version;
                }
            }

            /* ---------- Thrust ---------- */
            setValue('thrust.cal', cfg.thrust?.cal, 2);

            /* ---------- Torque ---------- */
            setValue('torque.cal1', cfg.torque?.cal1, 2);
            setValue('torque.cal2', cfg.torque?.cal2, 2);
            setValue('torque.distance_mm', cfg.torque?.distance_mm);

            /* ---------- Current ---------- */
            setValue('current.sensitivity', cfg.current?.sensitivity, 6);

            /* ---------- Voltage ---------- */
            setValue('voltage.calibration', cfg.voltage?.calibration, 3);

            /* ---------- Temperature ---------- */
            setValue('temp.coefficient', cfg.temp?.coefficient);
            setValue('temp.offset', cfg.temp?.offset);

            /* ---------- Motor PWM ---------- */
            setValue('motor.pwm_min_us', cfg.motor?.pwm_min_us);
            setValue('motor.pwm_max_us', cfg.motor?.pwm_max_us);
			setValue('motor.min_spin_percent', cfg.motor?.min_spin_percent);
			

            /* ---------- Motor Auto Disarm ---------- */
            setValue('motor.disarm_timeout_s', cfg.motor?.disarm_timeout_s);

            /* ---------- ESC Driver Type ---------- */
            setValue('motor.esc_driver_type', cfg.motor?.esc_driver_type);
			
            const label = escDriverToLabel(cfg.motor?.esc_driver_type);
            setValue('test.esc_driver', label);

			/* ---------- RPM ---------- */
            setValue('rpm.pulsesPerRevolution', cfg.rpm?.pulsesPerRevolution);
			setValue('rpm.updateRate', cfg.rpm?.responseMode);
			setValue('rpm.filterStrength', cfg.rpm?.filterLevel);
			setValue('rpm.smoothing', cfg.rpm?.smoothing);
			

        } catch (err) {
            console.error("Calibration WS update failed:", err);
        }
    });
}

export function setEscDriverEnabled(enabled){
	const el = document.getElementById("motor.esc_driver_type");
    if (!el) return;

    // Only allow changing if motor is disarmed (0)
    el.disabled = !enabled;
}

let escDrivers={};
let escDriversLableToId={};
	
	
export function escDriverToLabel(id) {
		return escDrivers[id]?.label ?? 'None';
	}
	
export function escDriverToId(label) {
		return escDriversLableToId[label];
	}
	
// Populate ESC driver select
function loadEscDriverOptions() {
	fetch('/api/calibration/escProtocols')
		.then(r => r.json())
		.then(data => {
			const sel = document.getElementById('motor.esc_driver_type');
			sel.innerHTML = '';
			data.esc_drivers.forEach(d => {
				escDrivers[d.id] = d;
				escDriversLableToId[d.label]=d.id;
				const opt = document.createElement('option');
				opt.value = d.id;
				opt.text = d.label;
				sel.add(opt);
			});
		});
}	



function loadRPMOptions() {
    fetch('/api/calibration/rpmOptions')
        .then(r => r.json())
        .then(data => {

            const opts = data.rpm.options;

            populateSelect('rpm.updateRate', opts.response);
            populateSelect('rpm.filterStrength', opts.filter);
            populateSelect('rpm.smoothing', opts.smoothing);

        })
        .catch(err => {
            console.error("RPM options load error", err);
            alert("Failed to load RPM options");
        });
}