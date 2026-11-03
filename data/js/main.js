// SPDX-FileCopyrightText: 2026 Peter Ludwig
// SPDX-License-Identifier: Apache-2.0

import * as WebSocket from './websocket.js';
import * as Network from './network.js';
import * as System from './system.js';
import * as Calibration from './calibration.js';
import * as Dirty from './dirty.js';
import * as Actions from './actions.js';
import * as Live from './live.js';
import * as UI from './ui.js';
import * as Safety from './safety.js';
import * as Test from './test.js';
import * as Protocol from './protocol.js';

const coreModules = [
    UI,
    System,
    Network,
    Calibration,
    Safety,
    Actions,
    Dirty
];

const featureModules = [
    Test,
    Protocol
];

const runtimeModules = [
    Live,        // transforms ws:data → live:update
    WebSocket    // source of events
];

// 1. Setup UI + base logic
coreModules.forEach(m => m.init());

// 2. Setup feature listeners
featureModules.forEach(m => m.init());

// 3. Start data flow LAST
runtimeModules.forEach(m => m.init());