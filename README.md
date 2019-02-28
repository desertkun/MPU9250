# MPU9250
MPU9250 (GY-91) driver for STM32 with HAL using SPI (spi1 by default).

## Setup
Define `GY_CS` pin in STM32CubeMX that will be used as Chip Select for the device, or pick the one you need in the `MPU9250_Config.h`.

## How To Use
Use `MPU9250_Init()` to initialize the device.

Then use this to retireve the raw data:
```c
int16_t AccData[3], GyroData[3], MagData[3];
MPU9250_GetData(AccData, GyroData, MagData);

printf("%08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d\n",
  (int16_t)AccData[0], (int16_t)AccData[1], (int16_t)AccData[2],
  (int16_t)GyroData[0], (int16_t)GyroData[1], (int16_t)GyroData[2],
  (int16_t)MagData[0], (int16_t)MagData[1], (int16_t)MagData[2]);
```
