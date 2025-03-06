# PINControlSystem.jl

A Julia module for controlling and managing PIN diodes via GPIO and SPI interfaces on Raspberry Pi 5. This system uses MAX6957 LED driver/GPIO expander chips to control multiple PIN diodes across multiple boards.

## Features

- Control of multiple daisy-chained MAX6957 chips via SPI
- GPIO and SPI communication with Raspberry Pi hardware
- Management of PIN diode states and current intensities
- Support for multiple control boards
- Mapping between PIN diodes and antenna connectors
- Configurable current settings for each PIN

## Dependencies

- DataFrames: For data management and manipulation
- Unitful: For working with physical units
- DimensionfulAngles: For angle measurements
- CSV: For reading configuration data
- PI5_LG_IO: Custom package for GPIO/SPI communication on Raspberry Pi 5

## Installation

1. Ensure you have Julia installed on your Raspberry Pi 5
2. Install the required dependencies:

```julia
using Pkg
Pkg.add(["DataFrames", "Unitful", "CSV"])
Pkg.add(url="https://github.com/jinsoo/DimensionfulAngles.jl")  # Replace with actual repository
Pkg.add(url="https://github.com/jinsoo/PI5_LG_IO.jl")  # Replace with actual repository
```

3. Clone this repository or add the module to your project

## Hardware Setup

This module is designed to work with:

- Raspberry Pi 5
- Multiple control boards with MAX6957 chips
- SPI connections (SPI0 and SPI1)
- Custom GPIO pins for SPI chip select
- PIN diode arrays

## Usage Example

```julia
using PINControlSystem

# Initialize GPIO
gpio_handle = Int64(lg_gpiochip_open(0))
if gpio_handle < 0
    error("Failed to open GPIO chip: $(lg_error_text(gpio_handle))")
end

# Initialize SPI0
spi0_handle = lg_spi_open(0, 0, 15_000_000, 0)
if spi0_handle < 0
    error("Failed to open SPI0: $(lg_error_text(gpio_handle))")
end

# Set up chip select pins
spi0_cs0_pin = 8  # GPIO pin 8 for SPI0_CS0
set_gpio_output(gpio_handle, spi0_cs0_pin)

# Create PIN controller
cs = PINController(gpio_handle)

# Add a board
put_board!(cs, 1, spi0_handle, spi0_cs0_pin)

# Configure the system
set_config(cs)

# Match antenna connectors with configuration file
matching_antenna_connectors!(cs)

# Control PIN states
put_pin_all_state!(cs, false)  # Set all pins to off
send_pin_states(cs)  # Send states to hardware

# Set specific PIN intensities
put_intensity!(cs, [1, 2, 3], [5, 10, 15])  # Set PINs 1, 2, 3 to different intensities
send_intensity_states(cs)  # Send intensity settings to hardware

# Clean up resources when done
lg_spi_close(spi0_handle)
lg_gpiochip_close(gpio_handle)
```

For a complete example, see the `example()` function in the code.

## API Reference

### Core Types

#### PINController
Main structure that manages all PIN configurations and states.

```julia
PINController(gpioH::Integer, riset::Unitful.ElectricalResistance=93.1u"kΩ")
```

### GPIO/SPI Management Functions

- `lg_gpiochip_open(gpio_chip)`: Open GPIO chip
- `lg_spi_open(bus, device, speed, mode)`: Open SPI device
- `set_gpio_output(gpio_handle, pin)`: Configure GPIO pin as output
- `lg_close()`: Close all GPIO and SPI handles

### Board Management

- `put_board!(cs, boardid, spi_handle, cs_pin)`: Add a board to the controller
- `set_config(cs)`: Configure all boards in the system
- `matching_antenna_connectors!(cs, filename)`: Map PIN diodes to antenna connectors from CSV

### PIN Control Functions

- `put_pin_all_state!(cs, state)`: Set all PIN states
- `put_pin_state!(cs, pid, state)`: Set specific PIN states by PIN ID
- `put_intensity!(cs, pinn, intensity)`: Set PIN intensities
- `send_pin_states(cs)`: Send all PIN states to hardware
- `send_intensity_states(cs)`: Send all intensity states to hardware

### Utility Functions

- `get_board_by_antconnector(cs, PINconnector)`: Get board by antenna connector
- `get_board_by_PINn(cs, PINn)`: Get board by PIN number
- `get_active_pins(cs)`: Get all active pins
- `get_bids(cs)`: Get all board IDs
- `get_board(cs, bid)`: Get board by ID

## Configuration File Format

The system uses a CSV file to map PIN diodes to antenna connectors. Expected columns include:

- Board number
- Port number
- SW number
- Antenna connector
- width(mm)
- length(mm)
- x1(mm), y1(mm), theta1(deg)
- x2(mm), y2(mm), theta2(deg)

## Hardware Details

### MAX6957 Configuration

The system configures MAX6957 chips with the following settings:
- Normal operation mode
- Individual current control
- Transition detection disabled
- Port configuration: GPIO output mode

## License

[Add license information here]

## Contributors

[Add contributor information here]