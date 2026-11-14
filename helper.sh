#! /bin/sh
# This helper script will automatically
#   Copy the specified UF2 file to the raspberry pico when it's boot drive is detected.
#   Start screen when the usb-serial is detected.

ACM="/dev/serial/by-id/usb-Raspberry_Pi_Pico_*"
BLK="/dev/disk/by-label/RPI-RP2"
UF2="$1"

[ -z "$UF2" ] && echo "UF2 file not specified, autoloader disabled."

while sleep 0.5; do
    [ -r $ACM ] && screen $ACM
    [ -e "$BLK"  -a -r "$UF2" ] && {
        echo "Mounting..."
        DIR=$(udisksctl mount -b "$BLK" | sed -rn 's/.* at //p')
        echo "Copying..."
        cp "$UF2" "$DIR"
        sync
        echo "Done."
    }
done
