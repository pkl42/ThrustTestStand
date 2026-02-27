
	function setValue(id, value, digits) {
		const el = document.getElementById(id);
		if (!el || value === undefined) return;
		el.value = (typeof digits === "number") ? value.toFixed(digits) : value;
	}
	
	function refreshFromButton(btn) {
		const fieldset = btn.closest('fieldset');
		const apiPath = fieldset?.dataset.api || btn.dataset.api;

		fetch(apiPath)
			.then(r => {
				if (!r.ok) throw new Error("Refresh failed");
				return r.json();
			})
			.then(data => {
				const scope = fieldset || document;

				scope.querySelectorAll('input').forEach(input => {
					if (data.hasOwnProperty(input.id)) {
						input.value = data[input.id];
					}
				});

				alert("Refreshed from device");
			})
			.catch(err => {
				alert("Error during refresh!");
				console.error(err);
			});
	}
	
	function postInputs(apiPath, inputs) {
		const payload = {};

		for (const input of inputs) {
			const path = input.id.split('.');
			let obj = payload;

			for (let i = 0; i < path.length - 1; i++) {
				if (!obj[path[i]]) obj[path[i]] = {};
				obj = obj[path[i]];
			}

			const value =
				input.type === 'number'
					? Number(input.value)
					: input.value;

			obj[path[path.length - 1]] = value;
		}

		// 🔑 Return the Promise
		return fetch(apiPath, {
			method: 'POST',
			headers: { 'Content-Type': 'application/json' },
			body: JSON.stringify(payload)
		})
		.then(r => {
			if (!r.ok) throw new Error("POST failed: " + r.status);
			// try to parse JSON, fallback to text
			return r.text().then(txt => {
				try { return JSON.parse(txt); } catch { return txt; }
			});
		})
		.catch(err => {
			console.error(err);
			alert("Failed to save settings");
			throw err; // propagate to handleAction .catch
		});
	}

	
	// Holds dirty flags for all sections
	const dirtyFlags = {};


	function updateDirtyUI(section) {
		const buttons = document.querySelectorAll(`button[data-section="${section}"]`);
		const isDirty = dirtyFlags[section];

		buttons.forEach(btn => {
			if (isDirty) {
				btn.classList.add('dirty'); // visual highlight only
			} else {
				btn.classList.remove('dirty');
			}
		});
	}
		
	function handleAction(btn) {
		const action = btn.dataset.action;
		const apiPath = btn.dataset.api;
		const rawParams = btn.dataset.params;

		let inputs = [];

		if (rawParams) {
			if (rawParams.endsWith('.*')) {
				// wildcard support, e.g. "limits.*"
				const prefix = rawParams.slice(0, -1); // "limits."
				inputs = Array.from(
					document.querySelectorAll(`input[id^="${prefix}"], select[id^="${prefix}"]`)
				);

			} else {
				// explicit list: a,b,c
				const ids = rawParams.split(',').map(s => s.trim());
				inputs = ids
					.map(id => document.getElementById(id))
					.filter(Boolean);
			}
		}

		if (action.includes('post') && inputs.length) {
			const section = inputs[0].dataset.section; // assume all inputs in same section
			
			console.log("POST inputs:", inputs.map(i => i.id));

			postInputs(apiPath, inputs)
				.then(() => {
					// only reset dirty flag if POST succeeded
					if (section) {
						dirtyFlags[section] = false;
						updateDirtyUI(section);
						// updateExecutionStatusUI(); // recompute isExecutable() if needed
					}
				})
				.catch(err => {
					console.error("Error posting data", err);
					alert("Failed to save settings. Changes not applied.");
				});
		}

		if (action.includes('refresh')) {
			if (btn.dataset.loader) {
				window[btn.dataset.loader]();
			} else {
				refreshFromButton(btn);
			}
		}
	}



	
	function toggleTheme() {
		const root = document.documentElement;
		const current = root.getAttribute("data-theme") || "dark";
		const next = current === "dark" ? "light" : "dark";
		root.setAttribute("data-theme", next);
		localStorage.setItem("theme", next);
	}

	(function initTheme() {
		const saved = localStorage.getItem("theme") || "dark";
		document.documentElement.setAttribute("data-theme", saved);
	})();
	
	

	function isExecutable() {
		const tripped = document.getElementById('safetyBanner').style.display === 'block';
		const dirty = dirtyFlags['safety']; // or iterate all relevant sections
		return !tripped && !dirty;
	}

	
	
	function postParam(apiPath, paramsObj, successMessage) {
		const params = new URLSearchParams();

		// Add each key-value pair from the object
		for (const [key, value] of Object.entries(paramsObj)) {
			params.append(key, value);
		}

		fetch(apiPath, {
			method: 'POST',
			headers: {
				'Content-Type': 'application/x-www-form-urlencoded'
			},
			body: params.toString()
		})
		.then(r => {
			if (!r.ok) throw new Error("Save failed");
			alert(successMessage);
		})
		.catch(err => {
			alert("Error during save!");
			console.error(err);
		});
	}
	
	
    /* -----------------------------
       CALIBRATION
    ------------------------------*/

	function loadCalibration() {
		fetch('/api/calibration')
			.then(r => {
				if (!r.ok) throw new Error("Calibration Values fetch failed");
				return r.json();
			})
			.then(cfg => {

				/* ---------- Version ---------- */
				if (typeof cfg.version === "string") {
					document.title = "Motor Thrust Test Stand " + cfg.version;

					const h1 = document.getElementById('page-title');
					if (h1) {
						h1.textContent = "Motor Thrust Test Stand: " + cfg.version;
					}
				}

				/* ---------- Thrust ---------- */
				setValue('thrust.cal', cfg.thrust?.cal, 2);

				/* ---------- Torque ---------- */
				setValue('torque.cal1',     cfg.torque?.cal1, 2);
				setValue('torque.cal2',     cfg.torque?.cal2, 2);
				setValue('torque.distance_mm', cfg.torque?.distance_mm);

				/* ---------- Current ---------- */
				setValue('current.sensitivity', cfg.current?.sensitivity, 6);

				/* ---------- Voltage ---------- */
				setValue('voltage.calibration', cfg.voltage?.calibration, 3);

				/* ---------- Motor PWM ---------- */
				setValue('motor.pwm_min_us', cfg.motor?.pwm_min_us);
				setValue('motor.pwm_max_us', cfg.motor?.pwm_max_us);
				
				/* ---------- Motor Auto Disarm ---------- */
				setValue('motor.disarm_timeout_s', cfg.motor?.disarm_timeout_s);
				
				/* ---------- ESC Driver Type ---------- */
				setValue('motor.esc_driver_type', cfg.motor?.esc_driver_type);
				const label = escDriverToLabel(cfg.motor?.esc_driver_type);

				setValue('test.esc_driver', label);
			})
			.catch(err => {
				console.error("loadCalibration failed:", err);
				alert("Failed to load calibration values");
			});
	}

	

    /* -----------------------------
       SAFETY LIMIT
    ------------------------------*/
	
	function enforceSafetyTrip() {
		const safetyBanner = document.getElementById('safetyBanner');
		const tripped = safetyBanner.style.display === 'block';

		const slider = document.getElementById('throttle');
		const startTestBtn = document.getElementById('test.start');

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

	
	function loadSafetySettings() {
        fetch('/api/safety')
        .then(r => {
            if (!r.ok) throw new Error("Safety Settings Values fetch failed");
            return r.json();
        })
        .then(data => {
			setValue('limits.throttle_percent', data.limits.throttle_percent, 2);
			setValue('limits.current_a',     data.limits.current_a, 2);
			setValue('limits.voltage_max_v',  data.limits.voltage_max_v, 2);
			setValue('limits.voltage_min_v',  data.limits.voltage_min_v, 2);
			setValue('limits.thrust_gf',     data.limits.thrust_gf, 2);
			setValue('limits.battery_cells', data.limits.battery_cells);
			
			const label = batteryPresetToLabel(data.limits.battery_cells);
			setValue('test.battery_cells', label);

        })
        .catch(err => {
            console.warn("Failed to load safety settings values:", err);
        });
    }


	// Manual reset Safety button
	function clearSafetyTrip() {
		fetch('/api/safety/clear', { method: 'POST' })
			.then(r => {
				if (!r.ok) throw new Error("Failed to clear safety trip");
				console.log("Safety trip cleared");
			})
			.catch(err => alert(err));
	}
	
	let batteryPresets = {};
	let batteryPresetLabelToId = {};
	
	function batteryPresetToLabel(presetId) {
	return batteryPresets[presetId]?.label ?? 'Custom / Not specified';
	}
	
	function batteryPresetToID(label) {
		return batteryPresetLabelToId[label] ?? null;
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
	
	function handleBatteryPresetChange(select) {
		const preset = batteryPresets[select.value];
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

function setVoltageLimits(preset) {
	const minInput = document.getElementById('limits.voltage_min_v');
	const maxInput = document.getElementById('limits.voltage_max_v');

	minInput.value = preset.voltage_min_v;
	maxInput.value = preset.voltage_max_v;

	minInput.dispatchEvent(new Event('input'));
	maxInput.dispatchEvent(new Event('input'));
}

    /* -----------------------------
       LIVE SECTION
    ------------------------------*/

	function resetThrottleUI(){
		document.getElementById('throttle').value = 0;
		document.getElementById('throttleVal').innerText = "0";
	}
	
	function onThrottleChange(value) {
    if (!isExecutable()) {
        alert("Motor disabled: safety trip active or unarmed or unsaved safety/configuation settings!");
        // reset slider to safe position
        const slider = document.getElementById('throttle');
        slider.value = 0;
        document.getElementById('throttleVal').textContent = '0';
        return;
    }

		document.getElementById('throttleVal').innerText = parseFloat(value).toFixed(1);
		setThrottle(parseFloat(value));
	}
	

	function setThrottle(val) {
		document.getElementById('throttleVal').innerText = val;

		fetch('/api/live/setThrottle', {
			method: 'POST',
			headers: {
				'Content-Type': 'application/x-www-form-urlencoded'
			},
			body: 'throttle=' + encodeURIComponent(val)
		});
	}

	function tare(event) {
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
   

    function updateLiveStatus() {
        fetch('/api/live/snapshot')
            .then(r => r.json())
            .then(data => {
                document.getElementById('thrust').innerText = data.thrust.toFixed(2);
                document.getElementById('torque').innerText = data.torque.toFixed(2);
				document.getElementById('torqueCell1').innerText = data.torqueCell1.toFixed(2);
				document.getElementById('torqueCell2').innerText = data.torqueCell2.toFixed(2);
                document.getElementById('voltage').innerText = data.voltage.toFixed(2);
                document.getElementById('current').innerText = data.current.toFixed(2);
                document.getElementById('power').innerText = data.power.toFixed(2);
				document.getElementById('thrust_ratio').innerText = data.thrust_ratio.toFixed(2);
                document.getElementById('rpm').innerText = data.rpm.toFixed(0);
                document.getElementById('temperature').innerText = data.temperature.toFixed(2);
                
                document.getElementById('throttleVal').innerText =
                    data.throttle_cmd.toFixed(1);
                document.getElementById('throttle').value =
                    data.throttle_cmd.toFixed(1);

			 // Safety
				const tripped = data.safety_tripped;
				const banner = document.getElementById('safetyBanner');
				if (tripped) {
					banner.style.display = 'block';
					document.getElementById('safetyTripSource').innerText = data.safety_trip_source;
					document.getElementById('safetyTripReason').innerText = data.safety_trip_reason;
					document.getElementById('safetyTripValue').innerText = data.safety_trip_value.toFixed(2);
					
					
				} else {
					banner.style.display = 'none';
					document.getElementById('safetyTripSource').innerText = '—';
					document.getElementById('safetyTripReason').innerText = '—';
					document.getElementById('safetyTripValue').innerText = '0';
				}
				enforceSafetyTrip();
	

            })
            .catch(err => {
                console.warn("Status fetch failed:", err);
            });
    }

    setInterval(updateLiveStatus, 500);

	/* -----------------------------
	   SYSTEM STATE MAPPING
	------------------------------*/

	const SensorStateMap = {
		0: { text: "Uninitialized", css: "state-uninit" },
		1: { text: "Initializing",  css: "state-init" },
		2: { text: "Ready",         css: "state-ready" },
		3: { text: "Error",         css: "state-error" },
		4: { text: "Disabled",      css: "state-disabled" }
	};

	function updateStateElement(id, stateValue) {
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
	
	function updateMotorButton(mountState) {
		const btn = document.getElementById("armMotorBtn");
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
	
	function toggleMotor(currentMountState) {
		const btn = document.getElementById("armMotorBtn");
		if (!btn) return;

		const mountState = currentMountState;

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
				// Refresh system state after action
				updateSystemState();
			})
			.catch(err => {
				console.error("Motor toggle failed:", err);
				btn.disabled = false; // re-enable on error
			});
	}


	function updateMountStateElement(id, stateValue) {
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
	
	function updateActuatorElement(id, stateValue) {
		const el = document.getElementById(id);
		if (!el || stateValue === undefined) return;

		const map = ActuatorStateMap[stateValue] ||
					{ text: "Unknown", css: "state-uninit" };

		el.textContent = map.text;
		el.className = "state " + map.css;
	}
	
	function updateSystemState() {
		fetch('/api/system')
			.then(r => r.json())
			.then(s => {

				updateStateElement("state-thrust", s.thrust);
				updateStateElement("state-torque", s.torque);
				updateStateElement("state-thermo", s.temperature);
				updateStateElement("state-rpm", s.rpm);
				updateStateElement("state-current", s.current);
				updateStateElement("state-voltage", s.voltage);

				updateActuatorElement("state-motor", s.motor);
				updateMountStateElement("state-motor-mount", s.motor_mount);
				
				latestMotorMount = s.motor_mount;      // store globally
				updateMotorButton(s.motor_mount);

			})
			.catch(err => {
				console.warn("System state fetch failed:", err);
			});
	}
	
	setInterval(updateSystemState, 1000);
	
	
	async function initializeSystem() {
		const response = await fetch("/api/system/initialize", { method: "POST" });
		const result = await response.json();

		alert(result.message);

		// Optionally refresh system state
		updateSystemState();
	}
	
	let escDrivers={};
	let escDriversLableToId={};
	
	
	function escDriverToLabel(id) {
		return escDrivers[id]?.label ?? 'None';
	}
	
	function escDriverToId(label) {
		return escDriversLableToId[label];
	}
	
	// Populate ESC driver select
	function loadEscDrivers() {
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


   	/* -----------------------------
	   TEST SECTION
	------------------------------*/
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

	function startErrorToString(code) {
		return StartError[code] || "unknown_error";
	}
	
	const TestStateMap = {
		0: { text: "Idle", css: "state-uninit" },
		1: { text: "StepInit",  css: "state-init" },
		2: { text: "StepRunning", css: "state-ready" },
		3: { text: "Completed", css: "state-ready" },
		4: { text: "Aborted", css: "state-error" },
		5: { text: "Failed", css: "state-error" }
	};
	



	let  lastTestRunning=null;

	function updateTestStatus() {
		fetch('/api/test')
			.then(r => r.json())
			.then(data => {
				if (data.running !== undefined) {

					const running = !!data.running;

					const map = TestStateMap[data.execState ?? 0] ;

					document.getElementById('test_status').innerText =map.text;

					document.getElementById('test_step').innerText =
						data.unitsDone ?? 0;

					document.getElementById('test_steps_total').innerText =
						data.unitsTotal ?? 0;
						
					document.getElementById('estDurationS').innerText =
						data.estDurationS.toFixed(0) ?? 0;						

					document.getElementById('actDurationS').innerText =
						data.durationS.toFixed(0) ?? 0;						

					const progress = Math.max(
						0,
						Math.min(100, data.progress ?? 0)
					);

					document.getElementById('testProgress').value = progress;
					document.getElementById('testProgressText').innerText =
						progress.toFixed(0) + " %";

					// visually dim when idle
					document.getElementById('testProgress').classList.toggle(
						'test-idle',
						!running
					);

					/* -------- Protocol Repository lock -------- */
					if (lastTestRunning !== running) {
						setRepoEnabled(!running);
						lastTestRunning = running;
					}
					
					document.getElementById('thrust_samples').innerText = data.thrust_samples  ?? 0;

				}
			})
			.catch(err => {
				console.warn("Status fetch failed:", err);
			});
	}

	setInterval(updateTestStatus, 1000);	

	document.addEventListener("DOMContentLoaded", () => {
		loadProtocols();
	});
	
	async function startTest() {
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

			/* 2) Reload config into UI */
			await loadTestConfig();

			/* 3) Start test */
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



    function stopTest() {
        fetch('/api/test/stop', { method: 'POST' });
    }

	function downloadMeanCSV()
	{
	    if (lastTestRunning) {
			alert("Please stop the test before downloading results.");
			return;
		}
		window.location.href = "/api/export/csv/mean";
	}

	function downloadStatsCSV()
	{
		  if (lastTestRunning) {
			alert("Please stop the test before downloading results.");
			return;
		}
		window.location.href = "/api/export/csv/statistics";
	}
	
	
	function loadTestConfig() {
		return fetch("/api/test/config")
			.then(r => {
				if (!r.ok) throw new Error("Test Metadata fetch failed");
				return r.json();
			})
			.then(cfg => {
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

				if (cfg.protocolID && cfg.protocolVersion != null ) {
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


	
   	/* -----------------------------
	   PROTOCOL SECTION
	------------------------------*/
		
	/**
	 * @typedef {Object} TestProtocolRef
	 * @property {string} id
	 * @property {string} version
	 */

	/** @type {TestProtocolRef | null} */
	let testProtocolSelection = null;
	let repoSelectedProtocol = null;
	
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

		
	function renderProtocolRepo(list) {
		const container = document.getElementById("protocol_list");
		container.innerHTML = "";

		list.forEach(p => {
			const entry = document.createElement("div");
			entry.className = "protocol-entry";
			entry.dataset.uid = `${p.id}@${p.version}`;
			entry.dataset.id = p.id;
			entry.dataset.version = p.version;

			entry.innerHTML = `
				<div class="protocol-name">${p.id}</div>
				<div class="protocol-meta">v${p.version}</div>
			`;

			entry.onclick = () =>
				selectProtocolRepo(
					{ id: p.id, version: p.version, file: p.file },
					entry
				);

			// visual selection only
			if (
				repoSelectedProtocol &&
				repoSelectedProtocol.id === p.id &&
				repoSelectedProtocol.version === p.version
			) {
				entry.classList.add("selected");
			}

			container.appendChild(entry);
		});

		// previously selected protocol vanished
		if (
			repoSelectedProtocol &&
			!list.some(p =>
				p.id === repoSelectedProtocol.id &&
				p.version === repoSelectedProtocol.version
			)
		) {
			repoSelectedProtocol = null;
			repoSelectedProtocol=null;
			clearProtocolRepoView();
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


	function loadProtocols() {
		fetch("/api/protocols")
			.then(r => {
				if (!r.ok) {
					return r.json().then(err => { throw err; });
				}
				return r.json();
			})
			.then(list => {
				if (!Array.isArray(list)) {
					throw new Error("Protocol list is not an array");
				}

				renderProtocolRepo(list);
				renderProtocolSelect(list);

				// 🔑 single source of truth
				resolveProtocolSelection();
			})
			.catch(err => {
				console.error("Failed to load protocols", err);
			});
	}



	function selectProtocolRepo(proto, entryEl) {
		// clone proto so nothing else can mutate it accidentally
		repoSelectedProtocol = {
			id: proto.id,
			version: proto.version,
			file: proto.file
		};

		// visual selection
		document.querySelectorAll(".protocol-entry")
			.forEach(e => e.classList.remove("selected"));
		entryEl.classList.add("selected");

		// load exact file (version-safe)
		loadProtocolDetails(repoSelectedProtocol.file);
	}

	

	function uploadProtocolToRepo() {
		const input = document.getElementById("protocol_file_repo");
		if (!input.files.length) return;

		const reader = new FileReader();
		reader.onload = () => {
			fetch("/api/protocols", {
				method: "POST",
				headers: { "Content-Type": "application/json" },
				body: reader.result
			})
			.then(r => r.json())
			.then(res => {
				if (res.error) {
					alert("Upload failed: " + res.error);
					input.value = "";
					return;
				}
				loadProtocols();
				alert(`Protocol uploaded: ${res.id} v${res.version}`);
				input.value = "";
			})
			.catch(err => {
				console.error("Upload error", err);
				alert("Upload failed");
				input.value = "";
			});
		};

		reader.readAsText(input.files[0]);
	}

	function setRepoEnabled(enabled) {
		document
			.querySelectorAll("#protocol-repository button, #protocol-repository input")
			.forEach(el => el.disabled = !enabled);
	}

		
	function deleteSelectedProtocolRepo() {
		if (!repoSelectedProtocol) {
			alert("No protocol selected");
			return;
		}

		if (!confirm(
			`Delete protocol ${repoSelectedProtocol.id} v${repoSelectedProtocol.version}?`
		)) return;

		const params = new URLSearchParams(repoSelectedProtocol);

		fetch(`/api/protocols?${params.toString()}`, {
			method: "DELETE"
		})
		.then(r => r.json())
		.then(res => {
			if (res.error) {
				alert("Delete failed: " + res.error);
				return;
			}

			repoSelectedProtocol = null;
			loadProtocols();
			clearProtocolRepoView();
			document.getElementById("test.protocol_info").textContent = "none";
		})
		.catch(err => {
			console.error("Delete error", err);
		});
	}

	
	function clearProtocolRepoView() {
		//document.getElementById("proto_name").textContent = "–";
		//document.getElementById("proto_steps").textContent = "–";
		//document.getElementById("proto_duration").textContent = "–";
		//document.getElementById("proto_checksum").textContent = "–";
		document.getElementById("protocol_json").textContent =
			"Select a protocol to view its JSON";
	}

	
	function loadProtocolDetails(file) {

		fetch(`/api/protocols/load?file=${encodeURIComponent(file)}`)
			.then(r => r.json())
			.then(proto => {

				// JSON view
				document.getElementById("protocol_json").textContent =
					JSON.stringify(proto, null, 2);
			})
			.catch(err => {
				console.error("Failed to load protocol", err);
				document.getElementById("protocol_json").textContent =
					"Failed to load protocol JSON";
			});
	}

			

		

    /* -----------------------------
       INIT
    ------------------------------*/
	window.onload = () => {
		resetThrottleUI();
		
		loadBatteryPresets();
		loadEscDrivers();
		
		loadCalibration(); // calibration + system config
		loadSafetySettings();
		loadTestConfig();
		
		
		// bind test protocol selection handler ONCE
		const protocolSelect = document.getElementById("test.protocol_select");
		protocolSelect.addEventListener("change", e => {
			const option = e.target.selectedOptions[0];
			applyProtocolSelection(option);
		});	


		// -----------------------------
		// Generic input/select change handler for dirty flags
		// -----------------------------
		document.querySelectorAll('input[data-section], select[data-section]').forEach(el => {
			const eventName = el.tagName === 'SELECT' ? 'change' : 'input';

			el.addEventListener(eventName, () => {
				const section = el.dataset.section;
				if (!section) return;

				dirtyFlags[section] = true;
				updateDirtyUI(section);
				// updateExecutionStatusUI(); // optional
			});
		});
		

		startPolling();
	};

	function startPolling() {
		updateLiveStatus();
		updateSystemState();
		updateTestStatus();
	}

