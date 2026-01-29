# Motor Thrust Test Stand – Test Protocols

This document explains how to define and use **test protocols** for the Motor Thrust Test Stand. Protocols are JSON documents that describe the sequence of steps, motion parameters, measurement configuration, and safety limits.

The goal of the protocol format is to be **safe by default**, **easy to read**, and **flexible where allowed**.

---

## 1. Protocol JSON Structure

A test protocol JSON has the following main sections:

```text
{
  "id": "<unique protocol ID>",
  "name": "<human-readable name>",
  "version": "<protocol version>",
  "estDurationS": 0,
  "unitsTotal": 0,
  "limits": { ... },
  "motion": { ... },
  "measurement": { ... },
  "steps": [ ... ]
}
```

---

## 2. Required vs Optional Fields

### 2.1 Required Fields

| Field | Type | Description |
|------|------|-------------|
| `id` | string | Unique identifier for the protocol |
| `name` | string | Human-readable name |
| `version` | string | Version of the protocol (e.g. `"1.0"`) |
| `steps` | array | Array of protocol steps |

---

### 2.2 Optional Fields and Default Resolution

All other sections (`limits`, `motion`, `measurement`) are **optional**.

They follow a strict **three-level resolution order**:

1. **Firmware defaults** (defined in the thrust stand firmware)
2. **Protocol header values** (top-level JSON fields)
3. **Step-level overrides** (inside individual steps)

If a field is omitted, the value from the previous level is used.

> **Important:**  
>
> - Firmware defaults always exist and are never undefined  
> - Step-level values only override the fields they explicitly define  

---

## 3. Safety Limits and Non‑Weakening Rules

Safety limits are treated differently from other optional fields.

### 3.1 Sources of Safety Limits

Safety limits can originate from three places:

1. **Firmware hard limits**  
   Absolute safety bounds compiled into the thrust stand firmware, which represents the hardware/sensor limits.

2. **Web UI safety configuration**  
   User-configured safety limits set before starting a test.

3. **Protocol limits**  
   Limits defined in the protocol JSON.

---

### 3.2 Non‑Weakening Principle (Critical)

Safety limits **can only become more restrictive**, never weaker.

This applies at every level:

- Protocol limits **cannot exceed firmware hard limits**
- Protocol limits **cannot exceed Web UI safety settings**
- Step-level limits **cannot exceed protocol limits**

If a protocol requests a weaker limit, the **more restrictive value wins silently**.

---

### 3.3 Limits Section

Defines maximum safe operating parameters:

```json
"limits": {
  "maxCurrentA": 30.0,
  "maxTempC": 80.0,
  "maxDurationS": 900,
  "maxVoltageV": 24.0,
  "minVoltageV": 18.0,
  "maxThrustGF": 5000.0
}
```

| Field | Description | Firmware Setting |
|------|-------------|-------------|
| `maxCurrentA` | Maximum allowed motor current (A) | 48 A, see Config.h CURRENT_SENSOR_MAX |
| `maxTempC` | Maximum allowed temperature (°C) | 120 C, see Config.h TEMPERATURE_SENSOR_MAX |
| `maxDurationS` | Maximum allowed test duration (seconds) | 2592000, representing 1 month |
| `maxVoltageV` | Maximum allowed supply voltage | 35 V, see Config.h VOLTAGE_SENSOR_MAX |
| `minVoltageV` | Minimum allowed supply voltage |-1 V, softcoded |
| `maxThrustGF` | Maximum allowed thrust (gram-force) | 5000 g, see Config.h THRUST_SENSOR_MAX |

> If any of these are omitted, the resolved value is taken from firmware or Web UI settings.

---

## 4. Motion Defaults and Overrides

### 4.1 Protocol-Level Motion Defaults

Defines default motion parameters for all steps:

```json
"motion": {
  "smooth": true,
  "accelTimeMs": 1000,
  "decelTimeMs": 1000,
  "settleTimeMs": 200
}
```

| Field | Description |
|------|-------------|
| `smooth` | Enable throttle smoothing |
| `accelTimeMs` | Acceleration time in milliseconds |
| `decelTimeMs` | Deceleration time in milliseconds |
| `settleTimeMs` | Optional delay after motion completes |

---

### 4.2 Step-Level Motion Overrides

Each step may override motion settings:

```json
"motion": {
  "smooth": false,
  "accelTimeMs": 500
}
```

- Only specified fields override protocol defaults  
- Missing fields inherit protocol-level values  

---

## 5. Measurement Defaults and Overrides

### 5.1 Protocol-Level Measurement Defaults

```json
"measurement": {
  "enabled": true,
  "completion": "onStop",
  "intervalS": 60,
  "windowMs": 2000
}
```

| Field | Description |
|------|-------------|
| `enabled` | Enable data recording |
| `completion` | `onStop`, `fixedDuration`, or `periodic` |
| `intervalS` | Period between samples (seconds) |
| `windowMs` | Measurement window duration (ms) |

---

### 5.2 Step-Level Measurement Overrides

```json
"measure": {
  "enabled": true,
  "completion": "fixedDuration",
  "windowMs": 1000
}
```

- Only overridden fields are replaced  
- Missing fields inherit protocol-level values  

---

## 6. Steps Section

Each protocol consists of an ordered array of steps:

```json
"steps": [
  { "type": "set", "throttle": 0, "dwellS": 2 },
  { "type": "ramp", "from": 0, "to": 100, "rate": 5 }
]
```

### 6.1 Supported Step Types

| Type | Required Fields | Description |
|------|-----------------|------------|
| `set` | `throttle`, `dwellS` | Set throttle and hold |
| `hold` | `throttle`, `dwellS` | Maintain throttle |
| `ramp` | `from`, `to`, `rate` | Continuous ramp |
| `stepSweep` | `from`, `to`, `step`, `dwellS` | Discrete sweep |

---

## 7. Example Protocol

```json
{
  "id": "throttle_sweep_0_100_5pct",
  "name": "Throttle Sweep 0–100% (5% steps)",
  "version": "1.0",

  "limits": {
    "maxCurrentA": 60,
    "maxTempC": 90
  },

  "motion": {
    "smooth": true,
    "accelTimeMs": 1000,
    "decelTimeMs": 1000
  },

  "measurement": {
    "enabled": true,
    "completion": "onStop",
    "windowMs": 2000
  },

  "steps": [
    {
      "type": "set",
      "throttle": 0,
      "dwellS": 2,
      "measure": { "enabled": false }
    },
    {
      "type": "stepSweep",
      "from": 0,
      "to": 100,
      "step": 5,
      "dwellS": 2,
      "measure": {
        "completion": "fixedDuration",
        "windowMs": 1000
      }
    }
  ]
}
```

---

## 8. Key Takeaways

- Firmware defaults always exist  
- Protocol headers define defaults for all steps  
- Steps override only what they explicitly define  
- Safety limits **never weaken**, only tighten  
- Unsafe values are automatically clamped  

This design keeps protocols expressive while guaranteeing safe operation at all times.
