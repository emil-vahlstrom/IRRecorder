# IRRecorder
Records and sends IR-remote signals. 
Don't expect to much from this project, it's a simple implementation that needs refactoring and is just meant to exist as a proof of concept. Perhaps it could give ideas how to send and receive IR-signals without being dependent on specific protocols and not using external libraries.

## Development environment
It's developed in Atmel Studio v.7.0.1931 

## Development board
This project has been developed and tested with a Genuino Micro (Arduino Micro) using the ATmega 32U4. 

## External Hardware 
External hardware is an infrared transmitter module (38KHz), IR-LED (940nm), keypad with 12 buttons (Accord AK-804-A-BBW), resistors (check datasheet for max-current on your hardware and choose appropriate resistors).

## Debugging hardware/software
* USB till RS232 converter (brand-name: FTDI) for reading serial input with PuTTY.
* Logic Analyzer used with Saleae Logic 1.2.18

## How to set up in Atmel Studio?
* Start a new Project, choose GCC C Executable Project (C/C++), name it IRRecorder
* Choose the chipset you're working it. It's only tested with Atmega 32U4 (i.e. Arduino Micro), but should work for most Arduino with a 16MHz CPU.
* To compile, add external tools: Tools -> External tools -> Add 
  * Title: <anything>
  * Command: C:\<path to avrdude>\avrdude.exe 
  * Arguments: -C "C:\<path to avrdude.conf>\avrdude.conf" -v -v -p atmega32u4 -c avr109 -P COM4 -b 57600 -D -U flash:w:"Debug\.hex":i

## Where to find avrdude.exe?
It should be located in the Arduino-folder, for example: C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avrdude.exe

## Where to find avrdude.conf?
It should be located in the Arduino-folder, for example: C:\Program Files (x86)\Arduino\hardware\tools\avr\etc\avrdude.conf
