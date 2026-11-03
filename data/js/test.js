// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

import {isExecutable,batteryPresetToID } from './safety.js';
import {bindElements} from './binder.js';
import {escDriverToId} from './calibration.js';


let testIsRunning=false;
let testProtocolSelection = null;

const TestStateMap = {
	0: { text: "Idle", css: "state-uninit" },
	1: { text: "StepInit",  css: "state-init" },
	2: { text: "StepRunning", css: "state-ready" },
	3: { text: "Completed", css: "state-ready" },
	4: { text: "Aborted", css: "state-error" },
	5: { text: "Failed", css: "state-error" }
};

const StartError = {
	0: "ok",
	1: "already_running",
	2: "missing_motor_type",
	3: "missing_propeller_type",
	4: "tare_failed",
	5: "invalid_protocol",
	6: "safety_locked",
	7: "MotorNotArmed"
};

export function init() {
    if (init.done) return;
    init.done = true;
	
	subscribeTestStatus();
	subscribeTestProtocolList();
	subscribeTestSettings();
    subscribeLiveUpdate();
	bindUI();
}


export async function startTest() {
		if (!testProtocolSelection) {
			alert("Please select a test protocol");
			return;
		}
		if (!isExecutable()) {
			alert("Motor disabled: safety trip active or unarmed or unsaved safety/configuation settings!");
			return;
		}
		const escDriverTypeId=escDriverToId(document.getElementById("test.esc_driver").value);
		const batteryTypeId= batteryPresetToID  (document.getElementById("test.battery_cells").value);

		const configPayload = {
			startTime: new Date().toISOString(),
			timezone: Intl.DateTimeFormat().resolvedOptions().timeZone,
			motorType: document.getElementById("test.motor_type").value,
			escType: document.getElementById("test.esc_type").value,
			propellerType: document.getElementById("test.prop_type").value,
			csvFormat: document.getElementById("test.csv_format").value,
			protocolID: testProtocolSelection.id,
			protocolVersion: testProtocolSelection.version,
			batteryCells: batteryTypeId,
			escDriverType: escDriverTypeId
		};

		try {
			/* 1) PUT config */
			const r = await fetch("/api/test/config", {
				method: "PUT",
				headers: { "Content-Type": "application/json" },
				body: JSON.stringify(configPayload)
			});

			const res = await r.json();
			if (!r.ok || res.error) {
				throw new Error(`Config failed: ${res.error || r.statusText}`);
			}

			/* 2) Start test */
			const startResp = await fetch("/api/test/start", {
				method: "POST",
				headers: { "Content-Type": "application/x-www-form-urlencoded" }
			});

			const startRes = await startResp.json();
			if (!startResp.ok || startRes.ok === false) {
				throw new Error(
					`Start failed: ${startErrorToString(startRes.error ?? -1)}`
				);
			}

			console.log("Test started successfully");
		} catch (err) {
			alert(err.message);
			console.error(err);
		}
	}

export function downloadMeanCSV()
	{
	    if (testIsRunning) {
			alert("Please stop the test before downloading results.");
			return;
		}
		window.location.href = "/api/export/csv/mean";
	}

export function downloadStatsCSV()
	{
		  if (testIsRunning) {
			alert("Please stop the test before downloading results.");
			return;
		}
		window.location.href = "/api/export/csv/statistics";
	}
	
function startErrorToString(code) {
	return StartError[code] || "unknown_error";
}

function resolveProtocolSelection() {
	const sel = document.getElementById("test.protocol_select");
	if (!sel || !testProtocolSelection) return;

	const value =
		`${testProtocolSelection.id}|${testProtocolSelection.version}`;

	const option = Array.from(sel.options)
		.find(o => o.value === value);

	if (option) {
		sel.value = value;
		applyProtocolSelection(option);
	} else {
		// selected protocol no longer exists → invalidate
		testProtocolSelection = null;
		sel.value = "";
	}
}

function renderProtocolSelect(list) {
	const sel = document.getElementById("test.protocol_select");
	sel.innerHTML = '<option value="">-- no protocol selected --</option>';

	list.forEach(p => {
		const opt = document.createElement("option");
		opt.dataset.uid = `${p.id}@${p.version}`;
		opt.dataset.id = p.id;
		opt.dataset.version = p.version;
		opt.value = `${p.id}|${p.version}`;
		opt.textContent = `${p.id} (v${p.version})`;
		sel.appendChild(opt);
	});
}


function updateTestRunningUI(){

	// visually dim when idle
	document.getElementById('testProgress').classList.toggle(
		'test-idle',
		!testIsRunning
	);
}

function applyProtocolSelection(option) {
		if (!option) {
			testProtocolSelection = null;
			return;
		}

		const { id, version } = option.dataset;
		if (!id || !version) {
			testProtocolSelection = null;
			return;
		}

		testProtocolSelection = { id, version };

		//document.getElementById("test.protocol_info").textContent =
		//	`ID: ${id}, Version: ${version}`;

		console.log("Test Protocol selected:", testProtocolSelection);
	}


function subscribeTestStatus() {
    document.addEventListener('ws:data', (e) => {
        const data = e.detail;

        if (data.type !== 'test_status') return;

        try {
            if (data.running === undefined) return;

            const map = TestStateMap[data.execState ?? 0];

            document.getElementById('test_status').innerText =
                map?.text ?? 'Unknown';

            document.getElementById('test_step').innerText =
                data.unitsDone ?? 0;

            document.getElementById('test_steps_total').innerText =
                data.unitsTotal ?? 0;

            document.getElementById('estDurationS').innerText =
                data.estDurationS?.toFixed(0) ?? 0;

            document.getElementById('actDurationS').innerText =
                data.durationS?.toFixed(0) ?? 0;

            const progress = Math.max(
                0,
                Math.min(100, data.progress ?? 0)
            );

            document.getElementById('testProgress').value = progress;
            document.getElementById('testProgressText').innerText =
                progress.toFixed(0) + " %";


        } catch (err) {
            console.warn("Test status WS update failed:", err);
        }
    });
}

function subscribeTestProtocolList() {
    document.addEventListener('ws:data', (e) => {
        const data = e.detail;

        if (data.type !== 'protocol_list') return;

        try {
            const list = data.protocols ?? [];

            if (!Array.isArray(list)) {
                throw new Error("Protocol list is not an array");
            }

            renderProtocolSelect(list);
            resolveProtocolSelection();

        } catch (err) {
            console.error("Failed to handle protocols WS update", err);
        }
    });
}

// as data.type == 'test_status' only pushed when test is running
// this subscription used for controlling buttons in test section
 
function subscribeLiveUpdate() {
    document.addEventListener('live:update', (e) => {
        const data = e.detail;

        testIsRunning = data.test_is_running ?? false;

        updateTestRunningUI();
    });
}

function bindUI() {
	    bindElements([
        {
            id: "test_start",
            event: "click",
            handler: startTest
		},
        {
            id: "test_export_meanCSV",
            event: "click",
            handler: downloadMeanCSV
		},	        
		{
            id: "test_export_statsCSV",
            event: "click",
            handler: downloadStatsCSV
		},
		{
            id: "test.protocol_select",
            event: "change",
            handler: ({ el }) => {
				const option = el.selectedOptions[0];
				applyProtocolSelection(option);
			}
		},

		
    ]);

}

function subscribeTestSettings() {
    document.addEventListener('ws:data', (e) => {
        const cfg = e.detail;

        if (cfg.type !== "test_config") return; // only handle test_config messages

        if (cfg.motorType) {
            document.getElementById("test.motor_type").value = cfg.motorType;
        }
        if (cfg.escType) {
            document.getElementById("test.esc_type").value = cfg.escType;
        }
        if (cfg.propellerType) {
            document.getElementById("test.prop_type").value = cfg.propellerType;
        }
        if (cfg.csvFormat) {
            document.getElementById("test.csv_format").value = cfg.csvFormat || ".,";
        }

        if (cfg.protocolID && cfg.protocolVersion != null) {
            if (!testProtocolSelection) {
                testProtocolSelection = {
                    id: String(cfg.protocolID),
                    version: String(cfg.protocolVersion)
                };
            }
            resolveProtocolSelection();
        }
    });
}