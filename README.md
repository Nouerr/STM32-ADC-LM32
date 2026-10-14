# STM32F407 Real-Time Temperature Monitoring System

## Overview
This project implements a real-time temperature monitoring system using an STM32F407 microcontroller and an LM35 analog temperature sensor.

The analog signal is acquired using the internal ADC, transferred to memory through DMA, and processed by the CPU. The measured temperature is displayed on a 16×2 LCD using the I2C communication protocol.

## Hardware
- STM32F407 Discovery board
- LM35 temperature sensor
- 16×2 LCD with I2C interface
- Breadboard and jumper wires

## Software and Peripherals
- STM32CubeIDE
- STM32CubeMX
- Embedded C / HAL
- ADC1, Channel 0 (PA0)
- DMA in circular mode
- I2C1: PB8 (SCL), PB9 (SDA)

## System Workflow
1. The LM35 converts temperature into an analog voltage.
2. ADC1 converts the analog signal into digital samples.
3. DMA transfers the samples into a memory buffer.
4. The CPU processes the acquired data and calculates the temperature.
5. The temperature is displayed on the LCD through I2C.

## Validation
- ADC acquisition and DMA memory transfer verified.
- Temperature display on the LCD tested successfully.

## Future Improvements
- Signal filtering and measurement accuracy evaluation.
- Comparison with a reference thermometer.
- Real-time temperature logging on a PC.
- Integration of additional analog sensors.
