#!/bin/bash

# This script will automatically mount a USB drive and run a .jar file from it.
# This is intended to be run as a service on the mini-pc to automatically startup photonvision.

# Define the UUIDs of your USB drives
USB_UUIDS=("2472-B4C1" "86A7-6CB6" "F058-D7CD")
MOUNT_POINT="/mnt/usbdrive"

# Create the mount point directory if it doesn't exist
mkdir -p $MOUNT_POINT

# Function to mount USB
mount_usb() {
    for UUID in "${USB_UUIDS[@]}"; do
        if blkid | grep -q $UUID; then
            echo "Mounting USB drive with UUID $UUID..."
            mount UUID=$UUID $MOUNT_POINT
            return 0
        fi
    done
    return 1
}

# Function to find and run the .jar file
run_jar() {
    # Find the .jar file on the mounted drive
    JAR_FILE=$(find $MOUNT_POINT -type f -name '*.jar' -print -quit)

    if [ ! -z "$JAR_FILE" ]; then
        # Change directory to where the JAR file is located
        JAR_DIR=$(dirname "$JAR_FILE")
        cd "$JAR_DIR"

        echo "Running .jar file from $JAR_DIR"
        java -jar "$JAR_FILE"
    else
        echo "No .jar file found on the USB drive."
    fi
}

# Attempt to mount the USB drive
if mount_usb; then
    # Check if mount was successful
    if mount | grep -q $MOUNT_POINT; then
        # Run the .jar file
        run_jar
    else
        echo "Failed to mount USB drive."
    fi
else
    echo "No known USB drive found."
fi
