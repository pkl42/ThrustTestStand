// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

export function setValue(id, value, digits) {
    const el = document.getElementById(id);
    if (!el || value === undefined) return;
    el.value = (typeof digits === "number") ? value.toFixed(digits) : value;
}

export function $(id) {
    return document.getElementById(id);
}

export function togglePassword(id) {
    const input = document.getElementById(id);
    if (!input) return;

    const isPassword = input.type === "password";
    input.type = isPassword ? "text" : "password";

    // optional UX improvement → update button icon
    const btn = document.querySelector(`[data-target-id="${id}"]`);
    if (btn) {
        btn.textContent = isPassword ? "🙈" : "👁";
    }
}

export function setIfExists(id, value) {
    if (value === undefined) return;

    const el = document.getElementById(id);
    // check if it's a form element
    if ("value" in el) {
        el.value = value; 
    } else {
        el.textContent = value; // for spans, divs, labels, etc.
    }
}


export function populateSelect(selectId, options, map = null) {
    const sel = document.getElementById(selectId);
    sel.innerHTML = '';

    options.forEach(optData => {
        if (map) map[optData.id] = optData;

        const opt = document.createElement('option');
        opt.value = optData.id;
        opt.text = optData.label;
        sel.add(opt);
    });
}