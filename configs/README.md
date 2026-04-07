# Configuration

The default configuration lives in `Default.yaml`. It is loaded by
`LegoBalance.RobotConfig.LoadConfig` when no override path is given.

## Overriding Values

You have three options.

1. **Local override file.** Create `configs/local.yaml` containing only the values you
   want to change. The loader will deep merge it on top of `Default.yaml`. The local file
   is gitignored.
2. **Explicit path.** Call `LoadConfig("path/to/your.yaml")` from your application code.
3. **Programmatic edits.** Load the default and mutate the returned `RobotConfig` before
   passing it to your subsystems. This is the best option for unit tests.

## Schema

The schema is documented inline in `Default.yaml`. The loader does not enforce a strict
schema in this scaffold release. It does validate that all required top level sections
exist and that numeric fields are numeric.

## Conventions

- Lengths are in meters, masses in kilograms, angles in radians.
- Wheel rates and gyro rates are in radians per second.
- Port names follow the SPIKE Prime labels A through F.
