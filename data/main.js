
	function setValue(id, value, digits) {
		const el = document.getElementById(id);
		if (!el || value === undefined) return;
		el.value = (typeof digits === "number") ? value.toFixed(digits) : value;
	}
	
	function postFromButton(btn) {
		const apiPath = btn.dataset.api;
		const paramIds = btn.dataset.params
			.split(',')
			.map(s => s.trim());

		const doRefresh = btn.dataset.refresh === "true";

		const params = new URLSearchParams();
		const errors = [];

		paramIds.forEach(id => {
			const el = document.getElementById(id);

			if (!el || el.value.trim() === "") {
				errors.push(`${id} is missing`);
				return;
			}

			const val = Number(el.value);
			if (!Number.isFinite(val)) {
				errors.push(`${id} is invalid`);
				return;
			}

			params.append(id, val);
		});

		if (errors.length) {
			alert(
				"Cannot save calibration:\n" +
				errors.map(e => `• ${e}`).join("\n")
			);
			return;
		}

		fetch(apiPath, {
			method: 'POST',
			headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
			body: params.toString()
		})
		.then(r => {
			if (!r.ok) throw new Error("Save failed");
			return r;
		})
		.then(() => {
			if (doRefresh) {
				refreshFromButton(btn);
			} else {
				alert("Value saved");
			}
		})
		.catch(err => {
			alert("Error during save!");
			console.error(err);
		});
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
	
	function handleAction(btn) {
		const action = btn.dataset.action;      // e.g., "post", "refresh", "post+refresh"
		const apiPath = btn.dataset.api;
		const paramIds = btn.dataset.params?.split(',').map(s => s.trim());

		if (action.includes('post') && paramIds) {
			postFromButton(btn); // your existing post helper
		}

		if (action.includes('refresh')) {
			// special-case: call dedicated loader if needed
			if (btn.dataset.loader) {
				window[btn.dataset.loader](); // e.g., "loadSafetySettings"
			} else {
				refreshFromButton(btn); // generic fieldset-based refresh
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
				setValue('motor_pwm.min_us', cfg.motor_pwm?.min_us);
				setValue('motor_pwm.max_us', cfg.motor_pwm?.max_us);
				
				/* ---------- Motor Auto Disarm ---------- */
				setValue('manual_idle_disarm_timeout_s', cfg.manual_idle_disarm_timeout_s);
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

		if (tripped) {
			// Reset UI
			slider.value = 0;
			document.getElementById('throttleVal').innerText = "0";

			// Prevent slider interaction
			slider.disabled = true;

		} else {
			// Safety cleared → re-enable slider
			slider.disabled = false;
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

    /* -----------------------------
       LIVE SECTION
    ------------------------------*/

	function resetThrottleUI(){
		document.getElementById('throttle').value = 0;
		document.getElementById('throttleVal').innerText = "0";
	}
	
	function onThrottleChange(value) {
		const tripped = document.getElementById('safetyBanner').style.display === 'block';
		if (tripped) {
			alert("Motor disabled: safety trip active!");
			return; // ignore input
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
				updateStateElement("state-thermocouple", s.thermocouple);
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
	
	function startTest() {
		if (!selectedProtocol) {
			alert("Please select a test protocol");
			return;
		}

		const motorType = document.getElementById("motor_type").value;
		const escType = document.getElementById("esc_type").value;
		const propType = document.getElementById("prop_type").value;
		const csvFormat = document.getElementById("csv_format").value;
		const protocolId = selectedProtocol.id;
		const protocolVersion = selectedProtocol.version;		

		/* ---------- 1) Configure run context ---------- */
		const configPayload = {
			motorType: motorType,
			escType: escType,
			propellerType: propType,
			csvFormat: csvFormat,
			protocolID: protocolId, 
			protocolVersion: protocolVersion
		};

		fetch("/api/test/config", {
			method: "PUT",
			headers: { "Content-Type": "application/json" },
			body: JSON.stringify(configPayload)
		})
		.then(async r => {
			const res = await r.json();
			if (!r.ok || res.error) {
				// include detail if available
				throw new Error(`Config failed: ${res.error || r.statusText}${res.detail ? " - " + res.detail : ""}`);
			}
			return res;
		})
		.then(() => {
			/* ---------- 2) Start test ---------- */
			return fetch("/api/test/start", {
				method: "POST",
				headers: { "Content-Type": "application/x-www-form-urlencoded" }
			});
		})
		.then(async r => {
			let res = {};
			try {
				res = await r.json();
			} catch (e) {
				// Non-JSON response
			}

			if (!r.ok || res.ok === false) {
				const errCode = res.error ?? -1;
				throw new Error(
					`Start failed: ${startErrorToString(errCode)}`
					+ (res.detail ? " - " + res.detail : "")
				);
			}

			console.log("Test started successfully");
		})
		.catch(err => {
			alert(err.message);
			console.error(err);
		});
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
	
	let pendingProtocolValue = null;
	
	function loadTestConfig() {
		fetch("/api/test/config")
			.then(r => {
				if (!r.ok) throw new Error("Test Metadata fetch failed");
				return r.json();
			})
			.then(cfg => {
				if (cfg.motorType) {
					document.getElementById("motor_type").value = cfg.motorType;
				}
				if (cfg.escType) {
					document.getElementById("esc_type").value = cfg.escType;
				}
				if (cfg.propellerType) {
					document.getElementById("prop_type").value = cfg.propellerType;
				}
				if (cfg.csvFormat) {
					document.getElementById("csv_format").value = cfg.csvFormat || ".,";
				}

				// remember protocol selection for later
				if (cfg.protocolID && cfg.protocolVersion != null) {
					pendingProtocolValue =
						`${cfg.protocolID}|${cfg.protocolVersion}`;

					resolveProtocolSelection();
				}
			})
			.catch(err => {
				console.warn("Failed to load Test Metadata:", err);
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
	let selectedProtocol = null;
	let userSelectedProtocol = false;
	
	function applyProtocolSelection(option) {
		if (!option || !option.dataset.id || !option.dataset.version) {
			selectedProtocol = null;
			return;
		}

		selectedProtocol = {
			id: option.dataset.id,
			version: option.dataset.version
		};
		

		document.getElementById("protocol_info").textContent =
			`ID: ${selectedProtocol.id}, Version: ${selectedProtocol.version}`;		

		console.log("Protocol selected:", selectedProtocol);
	}
	
	function resolveProtocolSelection() {
		const sel = document.getElementById("protocol_select");
		if (!sel || sel.options.length === 0) return;

		let optionToSelect = null;

		// 1) user selection wins
		if (userSelectedProtocol && selectedProtocol) {
			const value = `${selectedProtocol.id}|${selectedProtocol.version}`;
			optionToSelect = Array.from(sel.options)
				.find(o => o.value === value);
		}

		// 2) fallback to backend-loaded config
		if (!optionToSelect && pendingProtocolValue) {
			optionToSelect = Array.from(sel.options)
				.find(o => o.value === pendingProtocolValue);
		}

		if (optionToSelect) {
			sel.value = optionToSelect.value;
			applyProtocolSelection(optionToSelect);
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
				selectedProtocol &&
				selectedProtocol.id === p.id &&
				selectedProtocol.version === p.version
			) {
				entry.classList.add("selected");
			}

			container.appendChild(entry);
		});

		// previously selected protocol vanished
		if (
			selectedProtocol &&
			!list.some(p =>
				p.id === selectedProtocol.id &&
				p.version === selectedProtocol.version
			)
		) {
			selectedProtocol = null;
			clearProtocolRepoView();
		}
	}

		
	function renderProtocolSelect(list) {
		const sel = document.getElementById("protocol_select");
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
		selectedProtocol = {
			id: proto.id,
			version: proto.version,
			file: proto.file
		};

		// visual selection
		document.querySelectorAll(".protocol-entry")
			.forEach(e => e.classList.remove("selected"));
		entryEl.classList.add("selected");

		// load exact file (version-safe)
		loadProtocolDetails(selectedProtocol.file);
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
		if (!selectedProtocol) {
			alert("No protocol selected");
			return;
		}

		if (!confirm(
			`Delete protocol ${selectedProtocol.id} v${selectedProtocol.version}?`
		)) return;

		const params = new URLSearchParams(selectedProtocol);

		fetch(`/api/protocols?${params.toString()}`, {
			method: "DELETE"
		})
		.then(r => r.json())
		.then(res => {
			if (res.error) {
				alert("Delete failed: " + res.error);
				return;
			}

			selectedProtocol = null;
			loadProtocols();
			clearProtocolRepoView();
			document.getElementById("protocol_info").textContent = "none";
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
		
		loadCalibration();
		loadSafetySettings();
		loadTestConfig();
		
		// 🔑 bind protocol selection handler ONCE
		const protocolSelect = document.getElementById("protocol_select");
		protocolSelect.addEventListener("change", e => {
			const option = e.target.selectedOptions[0];
			userSelectedProtocol = true;   // user intent wins
			applyProtocolSelection(option);
		});		

		startPolling();
	};

	function startPolling() {
		updateLiveStatus();
		updateSystemState();
		updateTestStatus();
	}

