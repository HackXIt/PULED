# PULED

## Concept



## Milestones

- [ ] Project README
- [ ] Project concept
- [x] Protocol Definitions
- [ ] Protocol Implementation
   - [ ] Message Generation
   - [ ] Message Serialize
- [ ] Basic module functionality tested
   - [ ] Input Module
   - [ ] Output Module
- Project Tasks - Input Module
   - [ ] Task *"Gut"* => IN-02
   - [ ] Task *"Sehr gut"*  => IN-01
- Project Tasks - Output Module
   - [ ] Task *"Befriedigend"* => OUT-03
   - [ ] Task *"Gut"* => OUT-02
   - [ ] Task *"Sehr gut"* => OUT-01

## Tasks

### Input Module

> ### Für Erreichung bis zu einem „Gut“
>
> Pollen Sie die aktuellen Messwerte für die Pulsmessung (mit ca 50  Hz). Geben sie diese in SI-Einheiten weiter. Überlegen Sie sich eine sinnvolle pseudo-grafische Darstellung über  UART, bei der man den Pulsverlauf (pro Herzschlag) erkennen kann.
>
> ### Für Erreichung bis zu einem „Sehr gut“
>
> Benützen Sie den Interrupt des HeartRate Click-Moduls, um ein ständiges Pollen zu vermeiden. Werten Sie den tatsächlichen Herzschlag aus!
>
> **Achtung! - dieser Aufgabenteil geht in Richtung Signalverarbeitung**

### Output Module

> ### Für Erreichung bis zu einem „Befriedigend“
>
> Bei diesem Modul gibt es dafür keine eingeschränkte Aufgabe.
>
> ### Für Erreichung bis zu einem „Gut“
>
> Unterstützen Sie eine mehrzeilige Textausgabe auf dem OLED.
>
> ### Für Erreichung bis zu einem „Sehr gut“
>
> Unterstützen sie automatisches Scrollen mit einer Schrittweite von einem Pixel, wenn der Text größer als der Bildschirm ist.

## Side Notes

### Attaching a USB device in WSL

**Source:** https://devblogs.microsoft.com/commandline/connecting-usb-devices-to-wsl/

First ensure a WSL command prompt is open. This will keep the WSL 2 lightweight VM active.

From an administrator command prompt on Windows, run this command. It will list all the USB devices connected to Windows.

    usbipd wsl list

Select the bus ID of the device you’d like to attach to WSL and run this command. You’ll be prompted by WSL for a password to run a sudo command.

    usbipd wsl attach --busid <busid>

From within WSL, run lsusb to list the attached USB devices. You should see the device you just attached and be able to interact with it using normal Linux tools. Note that depending on your application, you may need to configure udev rules to allow non-root users to access the device.

Once you are done using the device in WSL, you can either physically disconnect the device or run this command from an administrator command prompt on Windows.

    usbipd wsl detach --busid <busid># PULED
