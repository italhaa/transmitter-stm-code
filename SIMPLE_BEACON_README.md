# Simple LoRa Beacon Transmitter

This is a simplified version of the STM32 LR1121 transmitter code that sends periodic beacon packets.

## Features

- Sends 150-byte packets every 3 seconds
- No response handling or complex protocol
- Simple packet structure with device ID, counter, timestamp, and pattern data
- LED indicators for transmission status
- Compatible with the Pi receiver setup

## Configuration

- **Frequency**: 2.444 GHz
- **Spreading Factor**: SF5
- **Bandwidth**: 500 kHz
- **Coding Rate**: 4/7
- **Sync Word**: 0x11
- **TX Power**: +2 dBm
- **Packet Size**: 150 bytes
- **Transmission Interval**: 3 seconds

## Packet Structure

| Offset | Size | Description |
|--------|------|-------------|
| 0-3    | 4    | Device ID (STM32 unique ID) |
| 4-7    | 4    | Packet counter |
| 8-11   | 4    | Timestamp (HAL_GetTick()) |
| 12-148 | 137  | Pattern data (index XOR counter) |
| 149    | 1    | Simple XOR checksum |

## Usage

1. Flash this code to your STM32 with LR1121 dev kit
2. The transmitter will start automatically
3. LEDs will indicate transmission status:
   - TX LED: Blinks during transmission
   - Status LED: Solid when system is active
4. Monitor serial output for transmission status

## Files Changed

- `Core/Src/main.c` - Updated to use simple beacon
- `Core/Src/lora_simple_beacon.c` - New simple transmitter implementation
- `Core/Inc/lora_simple_beacon.h` - Header file
- Original `lora_base.c` is replaced by the simple beacon

## Serial Output

The transmitter will output status messages via UART showing:
- Initialization status
- Device ID
- Packet transmission count

This simplified approach makes it easy to test and verify LoRa communication without complex protocol overhead.
