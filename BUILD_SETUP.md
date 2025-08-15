# STM32 LR1121 Project - VS Code Build Setup

This project has been successfully configured to build in VS Code instead of STM32CubeIDE.

## Prerequisites

The following tools have been installed:

1. **ARM GCC Toolchain**: `GNU Arm Embedded Toolchain 14.2.Rel1`
   - Location: `C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin`

2. **Make**: Uses the make utility from STM32CubeIDE
   - Location: `C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin`

3. **VS Code Extensions**:
   - C/C++ Extension Pack
   - Cortex-Debug
   - ARM Assembly

## Build Commands

### Using VS Code Tasks (Recommended)

1. **Build Project**: `Ctrl+Shift+P` → "Tasks: Run Task" → "Build STM32 Project"
2. **Clean Project**: `Ctrl+Shift+P` → "Tasks: Run Task" → "Clean STM32 Project"
3. **Build Release**: `Ctrl+Shift+P` → "Tasks: Run Task" → "Build Release"

### Using Terminal

```powershell
# Debug build (default)
make all

# Release build (optimized)
make DEBUG=0 all

# Clean build files
make clean

# Show build variables
make debug
```

## Output Files

After a successful build, you'll find these files in the `build/` directory:

- `LR1121.v4.elf` - The main executable file for flashing
- `LR1121.v4.hex` - Intel HEX format
- `LR1121.v4.bin` - Binary format
- `LR1121.v4.map` - Memory map file

## Flashing with STM32Programmer

Use the `LR1121.v4.elf` file to flash your STM32L476RG microcontroller:

```bash
STM32_Programmer_CLI -c port=SWD -w build/LR1121.v4.elf -v -rst
```

## Project Structure

- `Core/Src/` - Application source code
- `Core/Inc/` - Application headers
- `Drivers/` - STM32 HAL drivers and LR11xx driver
- `build/` - Build output directory
- `Makefile` - Build configuration

## Code Changes Made

1. Fixed circular GPIO pin definitions in `lora_simple_beacon.h`
2. Updated LR11xx driver API calls for current driver version
3. Added missing include for `lora_base.h` in `main.c`
4. Updated packet parameters structure to match current driver

## Debugging

If you have OpenOCD and ST-Link installed, you can use the "STM32 Debug" launch configuration for debugging.

## Notes

- The project is configured for STM32L476RG microcontroller
- Uses LR1121 LoRa transceiver driver
- Optimized for size in release builds (-Os)
- Debug symbols enabled in debug builds (-g3)
