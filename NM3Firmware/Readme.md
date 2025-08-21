# NM3 Firmware
This directory contains the firmware update utility for the NM3 modem provided by Succorfish. Please follow the instructions given in `NM3FlashingApplicationFirmware-20231018.pdf`, located in the same directory as this README.

## Firmware Updates
The bootloader on the modem is only active for the first 6 seconds after supplying power to it. To use the bootloader utility for updating firmware:

1. Open the Bootloader program and select the desired comm port.
2. Disconnect and reconnect the modem USB.
3. Wait 1 second for the USB to enumerate, then initiate the start command.

**Firmware Version:** NM3-1_6_0_UpdateForEndUser-20240215

