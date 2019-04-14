# IRRecorder
Records and sends IR-remote signals. 

Don't expect to much from this project, it's a simple implementation that needs refactoring and is just meant to exist as a proof of concept. Perhaps it could give ideas on how to send and receive IR-signals without being dependent on specific protocols and not using external libraries.

## Current features
What it can do now is to record an IR-signal, assign the recording to one of nine buttons and using the assigned button to "Play" the signal again. It's not dependent on any protocol, it just checks the time the signal has been on or off. It's tested by recording signals from a remote control (using the RC-5 protocol) and successfully using it on a TV.

## Future features (might never happen)
* Persist recorded commands on SRAM
* Reset previously recorded commands to default values
* Enter sleep mode after a few seconds and wake up on button press
* Use low power in sleep mode

### Development environment
The IDE used for this project is Atmel Studio v.7.0.1931.

### Development board
The dev-board used for testing the code is an Arduino/Genuino Micro board which uses the ATmega 32U4 chipset at a CPU-clock speed of 16MHz. 

### External Hardware 
External hardware is an infrared transmitter module (38KHz), IR-LED (940nm), keypad with 12 buttons (Accord AK-804-A-BBW), resistors (check datasheet for max-current of your hardware to choose appropriate resistor values).

### Debugging hardware/software
* USB to RS232 converter (brand-name: FTDI) for reading serial input with PuTTY or other serial monitors (Baud-rate is 19200)
* Logic Analyzer used with Saleae Logic 1.2.18

### How to set up project in Atmel Studio?
* Start a new Project -> Choose 'GCC C Executable Project (C/C++)' and name it 'IRRecorder' -> OK
* Choose the chipset you're working it, for example Atmega 32U4 (e.g Micro) or Atmega 328P (e.g Uno R3/Nano V3.0)
* Add external tools: Tools -> External tools -> Add 
  * Title: Arduino Flash Tool
  * Command: \<Arduino-folder\>\hardware\tools\avr\bin\avrdude.exe 
  * Arguments: -C "\<Arduino-folder\>\hardware\tools\avr\etc\avrdude.conf" -v -v -p atmega32u4 -c avr109 -P COM4 -b 57600 -D -U flash:w:"Debug\.hex":i

### How to upload code to Arduino?
* Build -> Build Solution
* (optional) Before flashing you may need to press the reset button on the Arduino for it to enter into the bootloader mode
* Tools -> Arduino Flash Tool -> OK

### Have any questions?
I'm probably not the right person to ask. Try to Google it.
