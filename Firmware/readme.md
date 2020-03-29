### This repository holds the firmware for a weather station based on the ESP32.

Original firmware and hardware were published in Elektor magazine: https://www.elektormagazine.com/magazine/elektor-70/42351  
Design by Roy Aarts.

The changes in this repo from the original design:  
- Removed SDS011 PM sensor and replaced by PMS5003 or [PM-E5 (hippocampus)](http://www.topsensor.cn/en/ChanPinZhanShi.html)
- Adapted project to platform.io
- prepared parameters for VEML6075 with PTFE window
- disabled UbiDots 
- WORK IN PRGRESS, does not compile yet, cleaning up code.
