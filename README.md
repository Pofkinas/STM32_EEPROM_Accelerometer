# STM32 HAL vibration level analysis

Using I2C pheriperal, MPU reads data from MPU6050 module, then over I2C stores it into AT24C256 module. Data displayed using PCF8574T I/O expansion module on 16x2 LCD screen. Simple Python script graphs data.

Used hardware:
* STM32L053R8T6
* LCD2X16
* PCF8574T
* MPU6050
* AT24C256
* Buttons
* Switch
* Rezistors

Completed on 5/31/2024, this marks my first project with STM32 MPU.
