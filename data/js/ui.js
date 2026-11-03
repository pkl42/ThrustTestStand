// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

export function init() {
    if (init.done) return;
    init.done = true;

	initTheme();
	bindUI();
}

export function toggleTheme() {
    const root = document.documentElement;
    const current = root.getAttribute("data-theme") || "dark";
    const next = current === "dark" ? "light" : "dark";
    root.setAttribute("data-theme", next);
    localStorage.setItem("theme", next);
}

function initTheme() {
    const saved = localStorage.getItem("theme") || "dark";
    document.documentElement.setAttribute("data-theme", saved);
}


function bindUI() {
    const bindings = [
        { id: "ui_toggle_theme", handler: () => toggleTheme() }
        // add more here as needed
    ];

    bindings.forEach(({ id, handler }) => {
        const btn = document.getElementById(id);
        if (!btn) return;
        btn.addEventListener("click", handler);
    });	

}