# PINControlSystem.jl

A Julia module for controlling and managing PIN diodes via GPIO and SPI interfaces on Raspberry Pi 5. This system uses MAX6957 LED driver/GPIO expander chips to control multiple PIN diodes across multiple boards.

## Features

- **Multi-Board Control**: Manage multiple daisy-chained MAX6957 chips via SPI.
- **Hardware Interface**: Direct GPIO and SPI communication with Raspberry Pi hardware.
- **PIN Diode Management**: Control state (ON/OFF) and current intensity (16 levels) for individual PIN diodes.
- **Flexible Configuration**: 
    - Global or Individual current control modes.
    - Display Test Mode for system verification.
    - Selective addressing for efficient updates in daisy chains.
- **Mapping System**: Map physical PIN diodes to logical antenna connectors using CSV configuration.

## Dependencies

- `DataFrames`: For data management and manipulation
- `Unitful`: For working with physical units
- `DimensionfulAngles`: For angle measurements
- `CSV`: For reading configuration data
- `PI5_LG_IO`: Custom package for GPIO/SPI communication on Raspberry Pi 5

## Installation

1. Ensure you have Julia installed on your Raspberry Pi 5.
2. Install the required dependencies:

```julia
using Pkg
Pkg.add(["DataFrames", "Unitful", "CSV"])
# Add custom packages (replace URLs with actual repositories if needed)
Pkg.add(url="https://github.com/jinsoo/DimensionfulAngles.jl")
Pkg.add(url="https://github.com/jinsoo/PI5_LG_IO.jl")
```

3. Clone this repository or add the module to your project.

## Hardware Setup

This module is designed to work with:
- **Raspberry Pi 5**
- Multiple control boards, each containing daisy-chained MAX6957 chips.
- **SPI Interfaces**: SPI0 and SPI1.
- **GPIO**: Custom pins used for Chip Select (CS) lines.

## Usage Example

```julia
using PINControlSystem

# 1. Initialize Hardware Handles
gpio_handle = Int64(lg_gpiochip_open(0))
spi0_handle = lg_spi_open(0, 0, 15_000_000, 0)

# 2. Configure Chip Select Pins
spi0_cs0_pin = 8
lg_set_gpio_output(gpio_handle, spi0_cs0_pin)

# 3. Initialize Controller
cs = PINController(gpio_handle)

# 4. Add Boards to System
# Board 1 on SPI0, CS pin 8
put_board!(cs, 1, spi0_handle, spi0_cs0_pin)

# 5. Load Configuration & Map Connectors
matching_antenna_connectors!(cs) # Loads from default CSV
set_config(cs) # Applies initial configuration to hardware

# 6. Control PINs
# Turn all PINs OFF
put_pin_all_state!(cs, false)
send_pin_states(cs)

# Set specific PINs ON
put_pin_state!(cs, [1, 2, 3], [true, true, true])
send_pin_states(cs)

# 7. Advanced Features
# Enable Display Test Mode (All LEDs Max Brightness)
set_display_test_mode(cs, true)

# Switch to Global Current Control and set level
set_current_mode(cs, :global)
set_global_current(cs, 8) # Set to mid-range intensity

# Selective Update (Advanced)
# Update specific chips on Board 1 without affecting others
send_spi_selective_chips(cs, 1, [1], [0x04], [0x01])

# 8. Cleanup
lg_close()
```

## API Reference

### Core Types
- `PINController`: Main system struct managing state and hardware handles.

### System Management
- `put_board!(cs, bid, spi, cs_pin)`: Add a board to the system.
- `set_config(cs)`: Apply configuration to all connected boards.
- `lg_close()`: Release all hardware resources.

### PIN Control
- `put_pin_all_state!(cs, state)`: Set state for ALL pins.
- `put_pin_state!(cs, pids, states)`: Set state for specific pins.
- `send_pin_states(cs)`: Commit pin states to hardware.
- `put_intensity!(cs, pids, intensities)`: Set intensity (0-15) for specific pins.
- `send_intensity_states(cs)`: Commit intensity settings to hardware.

### Advanced Features
- `set_display_test_mode(cs, enable)`: Toggle MAX6957 Display Test Mode.
- `set_current_mode(cs, mode)`: Switch between `:global` and `:individual` current control.
- `set_global_current(cs, level)`: Set global current level (0-15).
- `send_spi_selective_chips(cs, bid, cids, coms, vals)`: Send commands to specific chips in a daisy chain, using No-Op for others.

### Data Retrieval
- `get_board_by_antconnector(cs, name)`: Find board info by connector name.
- `get_active_pins(cs)`: Get list of currently active pins.

## Configuration File
The system expects a CSV file (default `data/link_SW_2_board.csv`) mapping logical antenna connectors to physical board/port addresses.

## License
[Add license information here]