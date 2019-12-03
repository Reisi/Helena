# Changelog
All notable changes to the NRF51 firmware will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

## [Unreleased]
### Added
### Changed
### Removed
### Fixed
### Known Issues

## [1.0.0] - 2019-12-03
### Added
- this changelog
- Daisy chain feature, lamp cannot only connect to remote but to other light, 
  too. Changes to another mode (either by remote or button) are relayed to 
  other connected lamps, this work bidirectional
- expanded light setups and added the ability to control an external taillight
  and brake indicator using the wired communication interface
- automatic wake-up from STANDBY mode when moving to speed up (re-)connection 
  to remote
- preferred mode added, when set to a valid mode, lamp will start into this 
  mode and, if shutdown via remote, return to this mode first
- option to calibrate driver board over debug interface

### Changed
- Switched mode configuration from enumeration to flag based setup
- changed temperature limiter to start later at 59°C (previous started at 47°C)
- changed voltage limiter to a two slope curve, to provide more output power at
  low battery temperatures
- user manual and quick start guide are adapted to flag based setup 
  configuration
- indicator leds are shut off in STANDBY mode to minimize current consumption
- debug interface changed, no "get error log" message necessary anymore, log is
  enabled automatically with enabling notifications

### Removed

### Fixed
- device will not bond to remote (had no effect, because remote initiated 
  bonding)
- garbage collection not working properly resulting in a lockup where no more 
  write operations are working.

### Known Issues
