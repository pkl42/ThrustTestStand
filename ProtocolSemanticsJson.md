{
  "id": "string",
  "name": "string",
  "version": "string",

  "limits": {
    "maxCurrentA": float,
    "maxTempC": float,
    "maxDurationS": uint
  },

  "motion": {               // protocol header defaults
    "defaultSmooth": bool,
    "stepAccelTimeMs": uint,
    "stepDecelTimeMs": uint,
    "settleTimeMs": uint
  },

  "measurement": {          // protocol header defaults
    "enabled": bool,
    "mode": "accumulate" | "timeseries" | "none",
    "intervalS": uint,      // for timeseries
    "windowS": uint         // for timeseries
  },

  "steps": [
    {
      "type": "set" | "hold" | "ramp" | "stepSweep",
      "throttle": float,          // set/hold
      "dwell": float,             // seconds
      "from": float,              // ramp
      "to": float,                // ramp
      "rate": float,              // ramp, %/s
      "measure": {                // optional, overrides header
         "enabled": bool,
         "mode": "...",
         "intervalS": uint,
         "windowS": uint
      },
      "motion": {                 // optional, overrides header
         "smooth": bool,
         "accelTimeMs": uint,
         "decelTimeMs": uint,
         "settleTimeMs": uint
      }
    }
  ]
}
