// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

import {bindElements} from './binder.js';

export function init() {
    if (init.done) return;
    init.done = true;
	subscribeProtocolList();
	subscribeLiveUpdate();
	bindUI();

}


/**
 * @typedef {Object} TestProtocolRef
 * @property {string} id
 * @property {string} version
 */

/** @type {TestProtocolRef | null} */

let repoSelectedProtocol = null;
	
	
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



export function selectProtocolRepo(proto, entryEl) {
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

	
export function uploadProtocolHandler({ el }) {
	
    if (!el.files || el.files.length === 0) {
        console.warn("No file selected");
        return;
    }

    const file = el.files[0];

    if (!(file instanceof Blob)) {
        console.error("Invalid file object:", file);
        return;
    }

    uploadProtocolToRepo(file, el);
}

export function uploadProtocolToRepo(file, inputEl) {
    const reader = new FileReader();

    reader.onload = () => {
        console.log("File read complete, sending to backend...");
        fetch("/api/protocols", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: reader.result
        })
        .then(r => r.json())
        .then(res => {
            if (res.error) {
                alert("Upload failed: " + res.error);
            } else {
                alert(`Protocol uploaded: ${res.id} v${res.version}`);
            }
            if (inputEl) inputEl.value = ""; // reset input
        })
        .catch(err => {
            console.error("Upload error", err);
            alert("Upload failed");
            if (inputEl) inputEl.value = "";
        });
    };

    reader.onerror = (err) => {
        console.error("FileReader error", err);
        alert("Failed to read file");
    };

    reader.readAsText(file);
}

function setRepoEnabled(enabled) {
		document
			.querySelectorAll("#protocol-repository button, #protocol-repository input")
			.forEach(el => el.disabled = !enabled);
	}

		
export function deleteSelectedProtocolRepo() {
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
			//loadProtocols();
			clearProtocolRepoView();
			//document.getElementById("test.protocol_info").textContent = "none";
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
	

function subscribeProtocolList() {
    document.addEventListener('ws:data', (e) => {
        const data = e.detail;

        if (data.type !== 'protocol_list') return;

        try {
            const list = data.protocols ?? [];

            if (!Array.isArray(list)) {
                throw new Error("Protocol list is not an array");
            }

            renderProtocolRepo(list);

        } catch (err) {
            console.error("Failed to handle protocols WS update", err);
        }
    });
}

function subscribeLiveUpdate() {
    document.addEventListener('live:update', (e) => {
        const data = e.detail;

        setRepoEnabled(!data.test_is_running);
    });
}
function bindUI() {
	    bindElements([
        {
            id: "protocol_delete_protocol",
            event: "click",
            handler: ({ el }) => {
                deleteSelectedProtocolRepo();
            }
		},
        {
            id: "protocol_file_upload",
            event: "change",
            handler: ({ el }) => {
                uploadProtocolHandler({el});
            }
		}
    ]);
}	