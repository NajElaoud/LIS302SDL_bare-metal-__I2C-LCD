# LIS302SDL_bare-metal-__I2C-LCD

This project demonstrates using the LIS302SDL accelerometer with an STM32 microcontroller to read X, Y, and Z acceleration data, display it on an I2C LCD, and control LEDs based on tilt direction. The code utilizes SPI and GPIO configurations for communication with the accelerometer, and I2C for LCD control.

## Features
- **Acceleration Measurement**: Reads X, Y, and Z axes from the LIS302SDL accelerometer.
- **Direction Detection**: Detects tilt direction and indicates movement (left, right, up, down) on LEDs.
- **LCD Display**: Outputs acceleration values and direction to an I2C-based LCD screen.

## Components Used
- **STM32 MCU** (model dependent on the board you're using)
- **LIS302SDL Accelerometer** - 3-axis accelerometer with SPI communication
- **I2C LCD Display** - 16x2 character LCD
- **GPIO LEDs** - Connected to display directional changes

## Project Structure
- **main.c**: Contains the main code for initializing peripherals and controlling accelerometer reading, display, and LED output.
- **i2c-lcd.c / i2c-lcd.h**: LCD display control library for STM32 (I2C communication).

## Getting Started

### Prerequisites
- **STM32CubeMX** and **STM32CubeIDE** for configuring and compiling code.
- **ST-Link** utility for flashing code onto the MCU.

### Setup and Wiring
1. **LIS302SDL Accelerometer**:
   - Connect **SPI** pins: PA5 (SCK), PA6 (MISO), PA7 (MOSI)
   - **CS** connected to GPIO pin PE3.
2. **I2C LCD Display**:
   - Connect **I2C** pins: PB6 (SCL) and PB7 (SDA)
3. **LEDs**:
   - Connect GPIO pins on Port D (PD12, PD13, PD14, PD15) for directional LEDs.

### Configuration
- **SPI1**: Configured for 8-bit data transfer with the accelerometer.
- **I2C1**: Configured for 100kHz for the LCD display.
- **GPIO**: Output for LEDs to indicate tilt direction.

### Code Overview
- `LIS302SDL_write()` and `LIS302SDL_read()` handle SPI communication with the accelerometer.
- `LIS302SDL_read_axis()` reads raw acceleration data for each axis.
- `Calibration()` converts raw axis data into calibrated values for display.
- **LED Control Logic**: Lights up specific LEDs based on tilt direction using acceleration values.
  
### Running the Project
1. Compile the code using **STM32CubeIDE**.
2. Flash the binary to your STM32 board using **ST-Link**.
3. Once powered, the LCD displays "LiS302DL_low_lvl" and the creator name briefly before switching to live axis readings.

### Output
- **LCD Display**: Displays real-time values of x, y, and z acceleration.
- **LEDs**: Indicates tilt direction:
  - **Left** - LED PD14
  - **Right** - LED PD12
  - **Up** - LED PD13
  - **Down** - LED PD15

## Example Output
**LCD**:
x=127 y=0
z=103 Dir: Left

**LEDs**:
- Corresponding LED lights up based on tilt direction.

---

Happy coding!
