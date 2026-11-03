# Changelog

All notable changes to the Motor Thrust Test Stand project will be documented in this file.

This project does not currently use formal releases.
Changes are documented chronologically.

---

## [Unreleased]

### Added

-

### Changed

-

### Fixed

-

### Removed

-


## [v2.3.0] 2026-03-28

### Added

- DShot ESC driver support seems to work (best experience with DShot300).  
  Parameter Min. Spin(%) added, to eleminate stackering on low throttle percentages.
- Temperature sensor calibration on Web UI  
- RPM sensor calibarion on Web UI, pulse detection changed to pcnt.
- Wifi Support for Stationary LAN.  
  Network setting can modified on Web UI.  
  For AP (Access Point) Mode, autodetection of best available channel, Wifi module sends on max. Power.  
  Added adjustable PIN (WIFI_AP_RESET) in Config, to reset AP settings to  
  SSID: ThrustStandAP Password: 12345678
  DNS Name Url-Mapping to: thruststand.local when connected into Local Network
- Websocket added to change communication between Web UI from pull requests to push events  
- Test Date and Time on test protocols


### Changed

- Refactored `ThrustStand::switchDriver()` to support dynamic driver configuration
- Updated pin configuration structure
- Resolve pin conflict CAGE_SWITCH_PIN, RPM_SENSOR_PIN when using MLX90614 (I2C) or MAX31855 (SPI)  
- js-files splitted based on Web UI sections (maintainable)

### Fixed

- bugs in  `ThrustStand::switchDriver()`




## [v2.2.0] 2026-02-27

### Added

- DShot ESC driver support (configurable RMT channel), but the whole DShot is not working as of now.
- MLX90614 I2C temperature sensor integration
- Sensor abstraction layer to integrate various sensors with the scope (TemperatureSensor)

### Changed

- Refactored `ThrustStand::switchDriver()` to support dynamic driver configuration
- Updated pin configuration structure
- Resolve pin conflict CAGE_SWITCH_PIN, RPM_SENSOR_PIN when using MLX90614 (I2C) or MAX31855 (SPI)

### Fixed

- index.hmtl, main.js were not in sync with github
- After E-Stop, system can be restarted
- Pullup/Pulldown pin logic on E-Stop corrected (inverted)

---
