// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

let ws;

export function init() {
    if (init.done) return;
    init.done = true;

	initWebSocket();

}

function initWebSocket() {
	
    ws = new WebSocket(`ws://${location.host}/ws`);

    ws.onopen = () => {
        console.log("WebSocket connected");
    };

    ws.onclose = () => {
        console.warn("WebSocket disconnected, retrying...");
        setTimeout(initWebSocket, 2000);
    };

    ws.onerror = (err) => {
        console.error("WebSocket error", err);
    };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);

            // 🔥 dispatch event globally
            document.dispatchEvent(
                new CustomEvent('ws:data', { detail: data })
            );
        } catch (e) {
            console.warn("Invalid WS message", event.data);
        }
    };
}