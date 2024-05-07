# For STM32CubeIDE:
### STM32 Connections:
- Temperature Sensor: SPI3
- Light Sensor: I2C1
- SD Card Reader: SPI2


### Code Defines:
- `#define BINARY_WRITE` writes in Binary format to the file. If `#define BINARY_WRITE` is commented out, it will write to the file in formatted text values.
- `#define DEBUG_PRINT` prints the value of the sensor every time. If `#define DEBUG_PRINT` is commented out, it will only print the time difference between readings. 
- `#define REALTIME` prints the value of the sensor data in a format for real time display. This is needed for the Python file to display the values in real time. 
  
### Current Features:
- Read Data from Sensors
- Write Data to SD Card
- Press User Button (Blue Button) to Unmount SD Card
- Blinking LED
- Print time difference in reading
- Plot Data using Matlab for text file
  - Real Time Visual Interface 

### Things to Do:
- Fix Temperature Data for C Code
- Write down the pinout to make life easier