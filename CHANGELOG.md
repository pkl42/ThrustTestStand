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

---

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
