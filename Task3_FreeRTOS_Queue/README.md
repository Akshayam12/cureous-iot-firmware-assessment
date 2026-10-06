
# Task 3 – FreeRTOS Queue Based Event Handling

## Objective

Implement a FreeRTOS-based event handling system using GPIO interrupts, queues, multiple tasks, and UART logging.

## Hardware

- STM32F446RE Nucleo Board
- User push button / external switch
- LED
- USB connection for serial terminal

## Software

- STM32CubeIDE
- STM32CubeMX
- FreeRTOS
- CMSIS-RTOS2
- STM32 HAL

## GPIO Configuration

### Button 1

```text
Pin: PC13
Button ID: 1

Pin: PB5
Button ID: 2

FreeRTOS Architecture

             GPIO Button
                  |
                  v
             EXTI ISR
                  |
                  v
            Event Queue
                  |
                  v
             Producer Task
                  |
                  v
             Log Queue
                  |
                  v
            UART Logger Task
                  |
                  v
            Serial Terminal

JSON

{
  "button_id": 1,
  "timestamp": 12543
}
