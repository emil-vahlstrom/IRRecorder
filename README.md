# IRRecorder
Records and sends IR-remote signals.
Developed in Atmel Studio v.7.0.1931. 
Don't expect to much from this project. Perhaps it could give ideas how to send and receive IR-signals without being dependent on specific protocols and not using external libraries.

## How to set up in Atmel Studio?
Start a new Project, choose GCC C Executable Project (C/C++), name it IRRecorder
Choose the chipset you're working it. It's only tested with Atmega 32U4 (i.e. Arduino Micro), but should work for most Arduino with a 16MHz CPU.

In order to compile, configure external tools:
In Atmel Studio press: Tools -> External tools -> Add 

Title: <anything>
Command: C:\<path to avrdude>\avrdude.exe 
Arguments: -C "C:\<path to avrdude.conf>\avrdude.conf" -v -v -p atmega32u4 -c avr109 -P COM4 -b 57600 -D -U flash:w:"Debug\.hex":i

## Where to find avrdude.exe?
It should be located in the Arduino-folder, for example: C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avrdude.exe

## Where to find avrdude.conf?
It should be located in the Arduino-folder, for example: C:\Program Files (x86)\Arduino\hardware\tools\avr\etc\avrdude.conf
