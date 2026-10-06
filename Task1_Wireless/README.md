# Task 1 – Wireless Connectivity using STM32 + ESP32

## Objective

Implement cloud connectivity using an STM32 microcontroller as the host and an ESP32 module as a Wi-Fi/MQTT modem using AT commands.

## Hardware

- STM32F446RE Nucleo Board
- ESP32 Wi-Fi Module
- USB connection for STM32 programming and serial terminal
- Jumper wires

## Software

- STM32CubeIDE
- STM32CubeMX
- STM32 HAL
- FreeRTOS / CMSIS-RTOS2
- ESP32 AT firmware

## UART Configuration

### USART2 – PC Serial Terminal

- Baud Rate: 115200
- Data: 8 bits
- Stop Bits: 1
- Parity: None

### USART1 – ESP32 Modem

- Baud Rate: 115200
- Data: 8 bits
- Stop Bits: 1
- Parity: None

## Features Implemented

- Dynamic Wi-Fi SSID and password configuration
- Wi-Fi credentials are not hardcoded in firmware
- ESP32 controlled through AT commands
- Wi-Fi connection using `AT+CWMODE` and `AT+CWJAP`
- MQTT cloud connection
- MQTT topic subscription
- MQTT message publishing
- 30-second heartbeat message
- Asynchronous UART reception
- Incoming MQTT cloud command handling
- Automatic Wi-Fi/MQTT reconnection
- FreeRTOS task-based architecture

## MQTT Heartbeat

The STM32 periodically publishes a heartbeat message every 30 seconds containing the device status and uptime.

Example:

```text
HEARTBEAT id=STM32_XXXX uptime=120 status=ONLINE

LED_ON
LED_OFF
LED_TOGGLE
PING
STATUS


FREERTOS Architecture

             ESP32
               |
             USART1
               |
        ESP RX Interrupt
               |
          RX Queue
               |
         ESP RX Task
               |
        MQTT Command
               |
          Command Queue
               |
          Command Task
               |
        STM32 GPIO / MQTT
