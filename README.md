# For STM32CubeIDE:
### STM32 Connections:
- Temperature Sensor: SPI3
- Light Sensor: I2C1
- SD Card Reader: SPI2


### Code Defines:
- `#define BINARY_WRITE` writes in Binary format to the file. Without `#define BINARY_WRITE` it will write to the file in formatted text values.
- `#define DEBUG_PRINT` prints the value of the sensor every time. Without `#define DEBUG_PRINT` it will only print the time difference between readings. 

### Current Features:
- Read Data from Sensors
- Write Data to SD Card
- Press User Button (Blue Button) to Unmount SD Card
- Blinking LED
- Print time difference in reading

### Things to Do:
- Fix Temperature Data
- Plot Data using Matlab/Python
- Real Time Visual Interface 
