# CANable Firmware - 33.333k Baud Modification

This is a fork of the original [CANable firmware](https://github.com/normaldotcom/canable-fw) with a modification to change the S2 command from 50k to 33.333k baud rate.

## Modification Details

- **S2 Command**: Now sets CAN bus to 33.333k baud (previously 50k)
- **Technical**: Changed prescaler from 120 to 180 for CAN_BITRATE_50K
- **Calculation**: 48MHz ÷ (180 × 8 time quanta) = 33,333 bps

This repository contains sources for the slcan CANable firmware, based off of the CANtact firmware. This firmware may still compile and run on the CANtact.

## Supported Commands

- `O` - Open channel 
- `C` - Close channel 
- `S0` - Set bitrate to 10k
- `S1` - Set bitrate to 20k
- `S2` - Set bitrate to 33.333k
- `S3` - Set bitrate to 100k
- `S4` - Set bitrate to 125k
- `S5` - Set bitrate to 250k
- `S6` - Set bitrate to 500k
- `S7` - Set bitrate to 750k
- `S8` - Set bitrate to 1M
- `M0` - Set mode to normal mode (default)
- `M1` - Set mode to silent mode
- `A0` - Disable automatic retransmission 
- `A1` - Enable automatic retransmission (default)
- `TIIIIIIIILDD...` - Transmit data frame (Extended ID) [ID, length, data]
- `tIIILDD...` - Transmit data frame (Standard ID) [ID, length, data]
- `RIIIIIIIIL` - Transmit remote frame (Extended ID) [ID, length]
- `rIIIL` - Transmit remote frame (Standard ID) [ID, length]
- `V` - Returns firmware version and remote path as a string

Note: Channel configuration commands must be sent before opening the channel. The channel must be opened before transmitting frames.

This firmware currently does not provide any ACK/NACK feedback for serial commands.

## Building

Firmware builds with GCC. On macOS, you can install the ARM toolchain using Homebrew:

```bash
brew install --cask gcc-arm-embedded
```

For other systems, download gcc-arm-none-eabi from [Launchpad](https://launchpad.net/gcc-arm-embedded/+download) and add the `bin` folder to your PATH.

### Compile the modified firmware:

```bash
git clone https://github.com/rjmcdougall/canable-33k.git
cd canable-33k
make
```

- If you have a CANable device, you can compile using `make`. 
- If you have a CANtact or other device with external oscillator, you can compile using `make INTERNAL_OSCILLATOR=1`

The built firmware will be in `build/canable-*.bin`

### Pre-built Binaries

Pre-built firmware binaries are available in the [Releases](https://github.com/rjmcdougall/canable-33k/releases) section.

## Flashing with the Bootloader

Simply plug in your CANable with the BOOT jumper enabled (or depress the boot button on the CANable Pro while plugging in). Next, type `make flash` and your CANable will be updated to the latest firwmare. Unplug/replug the device after moving the boot jumper back, and your CANable will be up and running.

## Debugging

Debugging and flashing can be done with any STM32 Discovery board as a
programmer, or an st-link. You can also use other tools that support SWD.

To use an STM32 Discovery, run [OpenOCD](http://openocd.sourceforge.net/) using
the stm32f0x.cfg file: `openocd -f fw/stm32f0x.cfg`.

With OpenOCD running, arm-none-eabi-gdb can be used to load code and debug.

## Contributors

- [Ethan Zonca](https://github.com/normaldotcom) - New features, HAL updates, Makefile fixes and code size optimization, updates for CANable
- [onejope](https://github.com/onejope) - Fixes to extended ID handling
- Phil Wise - Added dfu-util compatibility to Makefile

## License

See LICENSE.md
