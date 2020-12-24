#!/bin/bash
app_bin=T_FW_1_0_9.hex
bootloader_bin=slave_boot.hex
echo $app_bin
echo "start download"
#nrfutil settings generate --family NRF52 --application $app_bin --application-version 1 --bootloader-version 1 --bl-settings-version 1 settings.hex
nrfutil settings generate --family NRF52840 --application $app_bin --application-version 1 --bootloader-version 1 --bl-settings-version 1 settings.hex
mergehex --merge $bootloader_bin settings.hex --output bl_temp.hex
mergehex --merge bl_temp.hex $app_bin s140_nrf52_7.0.1_softdevice.hex  --output whole.hex
nrfjprog --eraseall -f NRF52 
nrfjprog --program whole.hex --verify -f NRF52 
nrfjprog --reset -f NRF52
echo "success"
