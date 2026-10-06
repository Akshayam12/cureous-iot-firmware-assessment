
Task2_ADC_UART/README.md`

```markdown
# Task 2 – ADC and UART

## Objective

Implement ADC data acquisition on the STM32 and transmit the converted ADC value through UART.

## Hardware

- STM32F446RE Nucleo Board
- Analog input source / potentiometer
- USB connection for serial communication

## Software

- STM32CubeIDE
- STM32CubeMX
- STM32 HAL

## Peripherals Used

### ADC

The ADC peripheral is configured to read an analog voltage from the selected ADC input pin.

The ADC converts the analog input into a digital value.

For a 12-bit ADC:

```text
ADC Range = 0 to 4095

ADC Value: 2048
ADC Value: 2180
ADC Value: 2315
ADC Value: 2450
