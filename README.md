# Attaching a device

First ensure a WSL command prompt is open. This will keep the WSL 2 lightweight VM active.

From an administrator command prompt on Windows, run this command. It will list all the USB devices connected to Windows.

    usbipd wsl list

Select the bus ID of the device you’d like to attach to WSL and run this command. You’ll be prompted by WSL for a password to run a sudo command.

    usbipd wsl attach --busid <busid>

From within WSL, run lsusb to list the attached USB devices. You should see the device you just attached and be able to interact with it using normal Linux tools. Note that depending on your application, you may need to configure udev rules to allow non-root users to access the device.

Once you are done using the device in WSL, you can either physically disconnect the device or run this command from an administrator command prompt on Windows.

    usbipd wsl detach --busid <busid># PULED
