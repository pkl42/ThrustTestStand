export function refreshFromButton(btn) {
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

export function postInputs(apiPath, inputs) {
    const payload = {};

    for (const input of inputs) {
        const path = input.id.split('.');
        let obj = payload;

        for (let i = 0; i < path.length - 1; i++) {
            if (!obj[path[i]]) obj[path[i]] = {};
            obj = obj[path[i]];
        }

        obj[path[path.length - 1]] =
            input.type === 'number' ? Number(input.value) : input.value;
    }

    return fetch(apiPath, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    })
    .then(r => {
        if (!r.ok) throw new Error("POST failed: " + r.status);
        return r.text().then(txt => {
            try { return JSON.parse(txt); } catch { return txt; }
        });
    });
}

