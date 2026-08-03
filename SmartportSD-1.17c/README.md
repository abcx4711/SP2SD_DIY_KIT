# SmartportSD version 1.17c
When switched on (when the adapter receives power from the Apple II) or after pressing the reset button on the microcontroller, the software loads up to 4 ProDOS images from the SD card and makes them available to an Apple II as drives via the SmartPort.
An active image can be used to boot the Apple II; the Eject button is used to change the boot image whilst the system is running.

## Defining the images using config.txt in the root directory of the SD card:
Multiple filenames can be entered line by line in the config.txt file. The software uses the first 4 files present on the SD card that are not empty. For example, if only two filenames are specified, only two drives will be available for the Apple II. Any more than four defined files, or empty or non-existent files, will be ignored.
Lines beginning with # are treated as comments. The maximum length of a filename must not exceed 60 characters.

## Defining images without config.txt:
If the config.txt file does not exist, the software attempts to open the files PART1.po, PART2.po, PART3.po and PART4.po in the root directory.

## General information about the images:
The files must be readable and not empty; otherwise, they will not be used. They must be unadorned ProDOS images. For .2mg files, the software checks whether they are in ProDOS sector order. If not, the image is ignored.

If the eject button is pressed several times within 2 seconds, the next image (1 – maximum 4) is selected for booting. At the end (after 2 seconds), the microcontroller restarts automatically. Shortly before the reboot, the microcontroller’s built-in LED triggers two flashing sequences.

Example 1 of the flashing pattern when Image 2 is selected: LED flashes twice, pause for 600 ms, LED flashes twice

Example 2 of the flashing pattern when Image 4 is selected: LED flashes four times, pause for 600 ms, LED flashes four times

On the Apple II, a reboot should then be triggered via SmartPort (for ROM4: press Control-Reset-Closed Apple, release Control-Reset and hold Closed Apple) or by switching the Apple II off and on.

## Error handling:
If the SD card is unreadable or an image could not be opened successfully, the microcontroller’s built-in LED flashes continuously in an endless loop without interruption.
Detailed error messages can be viewed via a connected serial terminal (speed is 230400, via the microcontroller’s USB port). If the SPIISD adapter is connected to the Apple II, it is important that the (+5V line) of the USB cable used (USB Power Cut Cable) is cut. Otherwise, this may cause damage to the Apple II!!

You can also use a SmartPhone with the "Serial USB Terminal App" a USB-OTG Adapter and a USB Power Cut Cable to view live log data of the adapter.

Character - is written for every 512 byte block read from the image and transfered to the Apple. Character + for every 512 byte block written to the sd image.

## Programming
For programming, the microcontroller is connected via USB, e.g. to a PC with urclock bootloader on the Nano 
On some Arduino Nanos, the microcontroller is a 328pb rather than a 328p. For programming, the correct ENV must be selected in the platformio.ini file. 
upload_speed is either 57600 or 115200; some Nano boards have problems with one or the other setting. It is important to ensure that the Nano is not connected to the Apple II!!

## In case the adapter won't work with an Apple II:
Disconnect it from the Apple and connect it via USB-Cable to a PC. Open a terminal (230400 baud) to see if something is wrong with the SD-Card or the images during program start in the log. At least loading of images and the eject button should work properly without an Apple.
