// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

import { postInputs } from './api.js';
import { setDirty, updateDirtyUI } from './dirty.js';

export function init() {
    if (init.done) return;
    init.done = true;
	
	bindUI();
}

/**
 * Handle a button or select action.
 * Supports:
 *  - POST inputs from form elements
 *  - Command-only POST
 *
 * @param {HTMLElement} btn - The button or select element
 */
export function handleAction(btn) {
    const { action, api: apiPath, params: rawParams, command } = btn.dataset;

    // Collect inputs if rawParams is defined
    let inputs = [];
    if (rawParams) {
        if (rawParams.endsWith('.*')) {
            const prefix = rawParams.slice(0, -1);
            inputs = Array.from(
                document.querySelectorAll(`input[id^="${prefix}"], select[id^="${prefix}"]`)
            );
        } else {
            const ids = rawParams.split(',').map(s => s.trim());
            inputs = ids
                .map(id => document.getElementById(id))
                .filter(Boolean);
        }
    }

    // Only handle POST actions for now
    if (!action || !action.includes('post')) return;

    if (inputs.length > 0) {
        // POST form inputs
        const section = inputs[0].dataset.section;

        console.log("POST inputs:", inputs.map(i => i.id));

        postInputs(apiPath, inputs)
            .then(() => {
                if (section) {
                    setDirty(section, false);
                    updateDirtyUI(section);
                }
            })
            .catch(err => {
                console.error("Error posting data", err);
                alert("Failed to save settings. Changes not applied.");
            });

    } 
	else if (command) {
		// Command-only POST
		btn.disabled = true; // disable button while request is in progress
		fetch(apiPath, { method: "POST" })
			.then(r => {
				if (!r.ok) throw new Error("Command failed");
				console.log("Command executed:", command);
				alert(`${command.charAt(0).toUpperCase() + command.slice(1)} executed successfully`);
			})
			.catch(err => {
				console.error(err);
				alert(`Command failed: ${command}`);
			})
			.finally(() => {
				btn.disabled = false; // re-enable button
			});
	}
}



/**
 * Initialize global click listener for all buttons/selects with data-action tag
 */
function bindUI() {
    document.addEventListener("click", (e) => {
        const el = e.target.closest("button, [data-action]");
        if (!el) return;

        // Generic API actions
        if (el.dataset.action) {
            handleAction(el);
            return;
        }
    });

}