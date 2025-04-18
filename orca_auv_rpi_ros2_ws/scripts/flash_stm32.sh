#!/bin/bash

st-flash --reset write ../stm32_binary/SAUVC2024.bin 0x08000000
