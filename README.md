# BNO055 I2C Driver

## Overview
This project provides a C++ driver for the **BNO055** IMU sensor, designed to communicate over the I2C protocol. The driver enables reading raw and processed sensor data, such as acceleration, gyroscope, magnetometer, Euler angles, quaternion values, and temperature.

## Features
- Supports I2C communication with the BNO055 sensor.
- Reads various sensor data, including:
  - Acceleration (m/s²)
  - Gyroscope (°/s)
  - Magnetometer (µT)
  - Euler angles (°)
  - Quaternion components
  - Linear acceleration (m/s²)
  - Gravity vector (m/s²)
  - Temperature (°C)
- Configurable operation modes.
- Proper error handling for I2C communication.

## Prerequisites
- Linux system with I2C support
- C++ compiler supporting C++11 or later
- CMake
- `make`

## Installation

### Clone the Repository
```sh
https://github.com/toghyan/bno055.git
```

### Build the Project
```sh
cd bno055
mkdir build
cd build
cmake ..
make
```

## Usage

### Running the Example
```sh
./bno055_example
```

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## Contributing
Feel free to open issues or submit pull requests to improve the driver.

## Author
[Ali Toghyan](https://github.com/toghyan)
