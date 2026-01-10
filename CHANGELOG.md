# MAXSwerve C++ Template Changelog

## [2026.0] - 2026-01-10

- Updates project for 2026 FRC season
- Uses REV Through Bore Encoder V2 preset for the absolute encoder config
- Updates kV (previously velocity feed forward) value for drive motor to adjust for feedforward changes from REVLib 2026
- Updates `PersistMode` and `ResetMode` imports
- Updates deprecated `setReference()` to `setSetpoint()`

## [2025.1] - 2025-01-04

- Updates project for kickoff releases
- Adds usage reporting

## [2025.0] - 2024-11-15

- Updates project for 2025 FRC season
- Uses new SPARK configuration mechanism introduced in REVLib 2025
- Removes slew rate limiter since it is no longer needed with MAXSwerve 2.0 wheels

## [2024.0] - 2024-01-08

- Updates project for 2024 FRC season

## [2023.1] - 2023-02-03

- Adds a configurable rate limiting system to prevent excessive loads from causing premature wheel failure
- Fixes turning SPARK MAX not using the correct feedback device

## [2023.0] - 2023-01-18

Initial release of MAXSwerve robot project
