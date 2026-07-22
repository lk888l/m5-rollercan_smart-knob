# RollerCAN I2C calibrator

Standalone Linux/Buildroot C++ tool that recalibrates the RollerCAN encoder
electrical-angle offset through the original firmware's I2C protocol.

The project layout follows `example_icm42688`: Linux bus access is under
`Peripheral`, the RollerCAN protocol is under `HardWare`, and the calibration
workflow is under `App`.

## Safety

Calibration energizes the motor at approximately 1.2 A phase current for about
three seconds. Remove all mechanical load, secure the unit, keep hands away from
the shaft, and preferably use a current-limited supply.

Do not enter the front-button SmartKnob setup mode while calibrating. RollerCAN
must be running the normal application in I2C mode.

## Wiring

Connect the Buildroot board and RollerCAN with a common ground:

| Buildroot board | RollerCAN Grove I2C |
|---|---|
| GND | GND / black |
| SDA, 3.3 V logic | SDA / yellow |
| SCL, 3.3 V logic | SCL / white |

Power RollerCAN separately through its supported power input. Do not add I2C
pull-ups to 5 V. The default 7-bit I2C address is `0x64`.

## Build on Windows with the reference ARM toolchain

The default configuration uses `arm-none-linux-gnueabihf-g++` and static
linking, matching the reference project:

```powershell
cmake -G "MinGW Makefiles" -S . -B build
cmake --build build --parallel
```

If the compiler is outside `PATH` and outside `C:/kk_software/toolchain`:

```powershell
cmake -G "MinGW Makefiles" -S . -B build `
  -DTOOLCHAIN_PATH=C:/path/to/toolchain/bin
cmake --build build --parallel
```

For a different Buildroot toolchain prefix or sysroot:

```powershell
cmake -G "MinGW Makefiles" -S . -B build `
  -DTOOLCHAIN_PATH=C:/buildroot/host/bin `
  -DTOOLCHAIN_PREFIX=arm-buildroot-linux-gnueabihf `
  -DSYSROOT_PATH=C:/buildroot/host/arm-buildroot-linux-gnueabihf/sysroot
cmake --build build --parallel
```

The output is:

```text
build/src/rollercan_i2c_calibrator
```

Release cross-builds are stripped by default to reduce the deployment size.
Pass `-DSTRIP_RELEASE_BINARY=OFF` when symbols are needed for debugging.

If the target toolchain has no static C/C++ runtime, configure with
`-DUSE_STATIC_LINKING=OFF` and copy the matching shared libraries to the target.

## Deploy and run

Copy the executable to the Buildroot board, then find its I2C device node:

```sh
ls -l /dev/i2c-*
```

First perform a non-motor probe:

```sh
chmod +x /tmp/rollercan_i2c_calibrator
/tmp/rollercan_i2c_calibrator --probe --bus /dev/i2c-3 --addr 0x64
```

Expected output includes:

```text
I2C communication OK; calibration_busy=0
```

Then run calibration. `--yes` is deliberately required because this operation
energizes the motor:

```sh
/tmp/rollercan_i2c_calibrator --yes --bus /dev/i2c-3 --addr 0x64
```

The tool performs the exact original-firmware sequence:

1. `00 00`: disable motor output.
2. `F1 01`: start encoder calibration.
3. Write `F3` with STOP, then separately read one byte until it changes from 1 to 0.
4. `F2 01`: apply and save the offset into RollerCAN flash.
5. `00 00`: leave output disabled.

After success, power-cycle RollerCAN and test with a low current limit and a low
positive/negative speed before applying a normal load.

## Troubleshooting

- `not_found`: wrong `/dev/i2c-*`, wrong address, missing common GND, or no ACK.
- `Permission denied`: run as root or adjust the i2c-dev node permissions.
- `calibration_not_started`: RollerCAN acknowledged I2C but did not enter
  calibration; check supply voltage, I2C mode, firmware, and whether the device
  is in the front-button setup menu.
- `timeout`: immediately remove power and inspect motor freedom, supply, and
  firmware before trying again.
