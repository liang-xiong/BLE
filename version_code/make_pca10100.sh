#!/bin/bash

Usage() {
    echo "Usage: $0 options type [veresion] [version] [version]"
    echo "  options:"
    echo "    -L : make listener.zip"
    echo "    -T : ble_peripheral"
    echo "  type:"
    echo "    -all : application bootloader and softdevice"
    echo "    -bl  : bootloader"
    echo "    -app : application"
    echo "  eg:"
    echo "    -all : bl_version app_hex_version app_version -all 1 1_0_8 8"
    echo "    -bl  : bl_version -bl 1"
    echo "    -app : app_hex_version app_version 1_0_8 8"
}

if [ "-all" == "$2" ] && [ $# -lt 5 ]; then
    Usage
    exit 1
fi
if [ "-app" == "$2" ] && [ $# -lt 4 ]; then
    Usage
    exit 1
fi
if [ "-bl"  == "$2" ] && [ $# -lt 3 ]; then
    Usage
    exit 1
fi

case $1 in

    -L)
    echo make ble_central listener.zip
    application=L_FW_
    bootloader=host_boot.hex
    output=listener.zip
    ;;

    -T)
    echo make ble_peripheral tag.zip
    application=T_FW_
    bootloader=slave_boot.hex
    output=tag.zip
    ;;

    *)
    Usage
    exit 1
    ;;
esac

case $2 in

    -all)
    echo bootloader $bootloader version $3
    echo application ${application}${4}.hex version $5
    echo softdevice s140_nrf52_7.0.1_softdevice.hex id 0xCA
    echo make all to $output
    nrfutil pkg generate --bootloader $bootloader --bootloader-version $3 --softdevice s140_nrf52_7.0.1_softdevice.hex --sd-id 0xCA --application ${application}${4}.hex --application-version $5 --hw-version 52 --sd-req 0xCA --key-file priv.pem $output
    ;;

    -bl)
    echo bootloader $bootloader version $3
    echo make bootloader to $output
    nrfutil pkg generate --bootloader $bootloader --bootloader-version $3 --hw-version 52 --sd-req 0xCA --key-file priv.pem $output
    ;;

    -app)
    echo application ${application}${3}.hex version $4
    echo make application to $output
    nrfutil pkg generate --application ${application}${3}.hex --application-version $4 --hw-version 52 --sd-req 0xCA --key-file priv.pem $output
    ;;

    *)
    Usage
    exit 1
    ;;  
esac

