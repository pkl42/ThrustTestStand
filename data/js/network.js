// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

import { setIfExists,togglePassword } from './helpers.js';
import {bindElements} from './binder.js';

export function init() {
    if (init.done) return;
    init.done = true;
	
	subscribeNetwork();
	bindUI();
}


function reconnectSTA() {
    fetch('/api/network/reconnect', { method: 'POST' });
}

function subscribeNetwork() {
    document.addEventListener('ws:data', (e) => {
        const cfg = e.detail;

        if (cfg.type !== 'network') return;

        try {
            setIfExists("network.ap_ssid", cfg.ap_ssid);
            setIfExists("network.ap_password", cfg.ap_password);
            setIfExists("network.ap_ip", cfg.ap_ip);
            setIfExists("network.ap_gateway", cfg.ap_gateway);
            setIfExists("network.ap_subnet", cfg.ap_subnet);

            setIfExists("network.sta_ssid", cfg.sta_ssid);
            setIfExists("network.sta_password", cfg.sta_password);
			
			setIfExists("network.sta_status", cfg.sta_state);
			

        } catch (err) {
            console.error("Network WS update failed:", err);
        }
    });
}


function bindUI() {
	    bindElements([
        {
            id: "network_toggle_ap_password",
            event: "click",
            handler: ({ el }) => {
                togglePassword(el.dataset.targetId);
            }
		},
        {
            id: "network_toggle_sta_password",
            event: "click",
            handler: ({ el }) => {
                togglePassword(el.dataset.targetId);
            }
		},	
        {
            id: "network_reconnect_sta",
            event: "click",
            handler: ({ el }) => {
                reconnectSTA();
            }
		}		
    ]);
}
 