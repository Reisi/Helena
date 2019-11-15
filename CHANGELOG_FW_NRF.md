# Changelog
All notable changes to the NRF51 firmware will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

## [Unreleased]
### Added
- this changelog
- Daisy chain feature, lamp cannot only connect to remote but to other light, 
  too. Changes to another mode (either by remote or button) are relayed to 
  other connected lamps, this work bidirectional
- expanded light setups and added the ability to control an external taillight
  and brake indicator using the wired communication interface

### Changed
- Switched mode configuration from enumeration to flag based setup
- changed temperature limiter to start later at 59°C (previous started at 47°C)
- changed voltage limiter to a two slope curve, to provide more output power at
  low battery temperatures
- user manual and quick start guide are adapted to flag based setup 
  configuration
- indicator leds are shut off in OFF mode to minimize current consumption

### Removed

### Fixed
- device will not bond to remote (had no effect, because remote initiated 
  bonding)

## [1.0.0] - 2017-06-20
