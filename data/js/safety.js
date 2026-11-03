// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

import { $,setValue } from './helpers.js';
import {isDirty } from './dirty.js';
import {bindElements} from './binder.js';

export function isExecutable() {
		const tripped = document.getElementById('safetyBanner').style.display === 'block';
		return !tripped && !isDirty('safety');
	}
	
export function init() {
    if (init.done) return;
    init.done = true;
	
	loadBatteryPresets();
	subscribeSafetySettings();
	subscribeLiveUpdate();
	bindUI();
}

// Manual reset Safety button
export function clearSafetyTrip() {
		fetch('/api/safety/clear', { method: 'POST' })
			.then(r => {
				if (!r.ok) throw new Error("Failed to clear safety trip");
				console.log("Safety trip cleared");
			})
			.catch(err => alert(err));
	}
	
let batteryPresets = {};
let batteryPresetLabelToId = {};
	
	
export function batteryPresetToLabel(presetId) {
	return batteryPresets[presetId]?.label ?? 'Custom / Not specified';
	}
	
export function batteryPresetToID(label) {
		return batteryPresetLabelToId[label] ?? null;
	}


export function enforceSafetyTrip() {
		const safetyBanner = document.getElementById('safetyBanner');
		const tripped = safetyBanner.style.display === 'block';

		const slider = document.getElementById('throttle');
		const startTestBtn = document.getElementById('test_start');

		if (tripped) {
			// Reset UI
			slider.value = 0;
			document.getElementById('throttleVal').innerText = "0";

			// Prevent slider interaction
			slider.disabled = true;
			startTestBtn.disabled=true;

		} else {
			// Safety cleared → re-enable slider
			slider.disabled = false;
			startTestBtn.disabled=false;
		}
	}
	
function loadBatteryPresets() {
	fetch('/api/safety/battery-presets')
		.then(r => r.json())
		.then(data => {
			const select = document.getElementById('limits.battery_cells');
			select.innerHTML = '';

			data.presets.forEach(preset => {
				batteryPresets[preset.id] = preset;
				batteryPresetLabelToId[preset.label] = preset.id;

				const opt = document.createElement('option');
				opt.value = preset.id;
				opt.textContent = preset.label;
				select.appendChild(opt);
			});
		});
}	
	
export function handleBatteryPresetChange({ el }) {
	const preset = batteryPresets[el.value];
	if (!preset) return;

	// NONE / Calibration preset
	if (preset.cells === 0) {
		if (confirm("Disable battery-based voltage limits?")) {
			setVoltageLimits(preset);
		}
		return;
	}

	if (confirm(
		`Apply default voltage limits for ${preset.label}?\n` +
		`Min: ${preset.voltage_min_v} V\n` +
		`Max: ${preset.voltage_max_v} V`
	)) {
		setVoltageLimits(preset);
	}
}

export function setVoltageLimits(preset) {
	const minInput = document.getElementById('limits.voltage_min_v');
	const maxInput = document.getElementById('limits.voltage_max_v');

	minInput.value = preset.voltage_min_v;
	maxInput.value = preset.voltage_max_v;

	minInput.dispatchEvent(new Event('input'));
	maxInput.dispatchEvent(new Event('input'));
}

function subscribeLiveUpdate() {
    document.addEventListener('live:update', (e) => {
        const data = e.detail;

        handleSafety(data);
    });
}

function handleSafety(data) {
    const banner = document.getElementById('safetyBanner');

    if (data.safety_tripped) {
        banner.style.display = 'block';

        document.getElementById('safetyTripSource').innerText =
            data.safety_trip_source ?? '—';

        document.getElementById('safetyTripReason').innerText =
            data.safety_trip_reason ?? '—';

        document.getElementById('safetyTripValue').innerText =
            data.safety_trip_value?.toFixed(2) ?? '-';
    } else {
        banner.style.display = 'none';

        document.getElementById('safetyTripSource').innerText = '—';
        document.getElementById('safetyTripReason').innerText = '—';
        document.getElementById('safetyTripValue').innerText = '0';
    }

    enforceSafetyTrip();
}


	
function subscribeSafetySettings() {
    document.addEventListener('ws:data', (e) => {
        const data = e.detail;

        if (data.type !== 'safety_settings') return;

        try {
            const limits = data.limits || {};

            setValue('limits.throttle_percent', limits.throttle_percent, 2);
            setValue('limits.current_a', limits.current_a, 2);
            setValue('limits.voltage_max_v', limits.voltage_max_v, 2);
            setValue('limits.voltage_min_v', limits.voltage_min_v, 2);
            setValue('limits.thrust_gf', limits.thrust_gf, 2);
            setValue('limits.battery_cells', limits.battery_cells);

            const label = batteryPresetToLabel(limits.battery_cells);
            setValue('test.battery_cells', label);

            setValue('limits.temperature_c', limits.temperature_c, 0);

        } catch (err) {
            console.warn("Safety settings update failed:", err);
        }
    });
}


function bindUI() {
	bindElements([
	{
		id: "limits.battery_cells",
		event: "change",
		handler: ({ el }) => {
			handleBatteryPresetChange({el});
		}
	}
]);
}