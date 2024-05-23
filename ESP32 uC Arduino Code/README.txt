Need the following:

1. ESP32-DevkitC V4 or similar board
  - Datasheet: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html 

2. CP210x USB to UART Bridge VCP Driver
  - This is required for your computer to detect the ESP32 board
  - Usually is through port: COM7 or COM3
  - Download: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads 

4. Arduino IDE 
  - Board Manager: esp32 by Espressif Systems
      - In the "Board:" dropdown menu, select: "ESP32 Dev Module"
  - Libraries: arduinoFFT v1.62 by Enrique Condes, version 1.62 is highly important
