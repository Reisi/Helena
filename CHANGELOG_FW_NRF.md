# Changelog
All notable changes to the NRF51 firmware will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

## [3.0.0-alpha] - unreleased
### Changed
v3 is not a continued development, it is the nrf52 based helen project ported to the nrf51.
- added support for a second central connection
- channels can be configured completely individual, making the need for different firmware variants obsolete
- the com pin can alternatively be used for an external button
- limiter is now prioritized, with spot having a higher priority as the flood

## [2.3.1] - unreleased
### Added

### Changed
- mode changes from com will now be relayed

### Removed

### Fixed
- fixed wrong version build format
- database discovery will be restarted if first attempt is busy

### Known Issues
- daisy chain: if quickly changing mode twice, the second change may not be 
  executed by the remote lamp.

## [2.3.0] - 2022-07-24
### Added
- support for billy base led driver
- bonding required to change settings (only for release)
- support for avisio/pearl remote control

### Changed
- group size can be configured individually

### Removed

### Fixed
- missing length checking for debug commands
- false decoding for com master messages
- going backwards to modes and groups malfunction
- missing line breaks in debug calibration section
- watchdog reset if transferring long error log over BLE

### Known Issues
- daisy chain: if quickly changing mode twice, the second change may not be 
  executed by the remote lamp.

## [2.2.1] - 2021-05-30
### Added

### Changed

### Removed

### Fixed
- wrong placed error check on data storage
- billina not transmitting correct high beam intensity
- features incomplete
- rounding error at intensity converting
- wrong error code if sensor offset not done yet

### Known Issues
- daisy chain: if quickly changing mode twice, the second change may not be 
  executed by the remote lamp.

## [2.2.0] - 2021-05-02
### Added
- improved status led, blue blinking while scanning, red and blue active works
  now for HWREV22 and HWREV23, too.
- added battery SOC and taillight power to measurement notification
- temporary mode
- hold button for 10 sec. (from off state) will initiate factory reset
- SOS mode for Helena, to enter press button for 10 sec. from an active mode
- support for 1- and 3-cell li-ion batteries.

### Changed

### Removed

### Fixed
- BLE_GATT_STATUS_ATTERR_CPS_PROC_ALR_IN_PROG leading to reset
- automatic led count detection might deliver wrong led count

### Known Issues
- daisy chain: if quickly changing mode twice, the second change may not be 
  executed by the remote lamp.


## [2.1.0] - 2021-04-18
### Added
- support for R51 "The Lord of Rings" Remote Control

### Changed
- behavior of preferred mode
- removed service discovery for HID service and replaced it with hardcoded 
  presets.

### Removed

### Fixed
- several fixes concerning data storage

### Known Issues
- automatic led count detection might deliver wrong led count


## [2.0.2] - 2021-03-30
### Added

### Changed

### Removed

### Fixed
- mode request not using full message length
- wrong flags in measurement data

### Known Issues
- automatic led count detection might deliver wrong led count

## [2.0.1] - 2021-03-28
### Added

### Changed

### Removed

### Fixed
- changing between helena and billina firmware now possible without loss of 
  settings

### Known Issues
- automatic led count detection might deliver wrong led count


## [2.0.0] - 2021-03-25
### Added
- added build target for "Billy" mode, a setup especially for bike lights 
  without pitch compensation, but separate driver current for low and high 
  beam.

### Changed
- light control services was modified and extended to meet requirements for 
  bicycle light

### Removed

### Fixed
- update from version 0.X will not end in an error loop anymore

### Known Issues
- when changing for helena to billina firmware (and vice versa) settings will 
  reset to defaults
- automatic led count detection might deliver wrong led count


## [1.1.0] - 2020-03-11
### Added
- re-implemented reset survive (lamp keeps its mode even if it is reset by 
  watchdog)
- release version

### Changed
- reworked debug interface

### Removed

### Fixed

### Known Issues
- Update from version 0.X might result in an infinite error loop. 
  Workaround: flash back to old firmware and perform factory reset, then update
  will work
- automatic led count detection might deliver wrong led count

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
- Update from version 0.X might result in an infinite error loop. 
  Workaround: flash back to old firmware and perform factory reset, then update
  will work
