// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

const dirtyFlags = {};

export function init(){
    if (init.done) return;
    init.done = true;
	
	initDirtyHandlers();
}

export function updateDirtyUI(section) {
    const buttons = document.querySelectorAll(`button[data-section="${section}"]`);
    const isDirty = dirtyFlags[section];

    buttons.forEach(btn => {
        btn.classList.toggle('dirty', isDirty);
    });
}

export function setDirty(section, value) {
    dirtyFlags[section] = value;
    updateDirtyUI(section);
}

export function isDirty(section) {
    return !!dirtyFlags[section];
}


function initDirtyHandlers() {
    document.querySelectorAll('input[data-section], select[data-section]')
        .forEach(el => {
            const eventName = el.tagName === 'SELECT' ? 'change' : 'input';

            el.addEventListener(eventName, () => {
                const section = el.dataset.section;
                if (!section) return;

                dirtyFlags[section] = true;
                updateDirtyUI(section);
            });
        });
}
