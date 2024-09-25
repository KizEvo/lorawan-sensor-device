# Getting Started

Sensor device source code for LoRaWAN system.

This repository contains two branches:
- The `main` branch contain original implementation of AES128 and AES-CMAC encryption scheme.
- The `ascon-encryption` branch contain the AES128 and Asconmacav12 encryption scheme

AES128 is used to encrypt the LoRaWAN payload (the actual data).
AES-CMAC and Asconmacav12 is used to authenticate the whole LoRaWAN package.

This project used KeilC Uvision 5. It's recommended that you only get the necessary drivers and import it to your project. This means you should create your own project and only get the drivers from here.

The microcontroller in used is STM32F103C8T6. Current sensor module is the temperature/humidity DHT11. The LoRa transceiver is the SX1276 868/915MHz on Ra-01 breakout board.
