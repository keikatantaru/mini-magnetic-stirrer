NAME := avr_motor_driver

build:
	avr-gcc -Wall -g -Os -mmcu=attiny85 -o $(NAME).bin $(NAME).c
	avr-objcopy -j .text -j .data -O ihex $(NAME).bin $(NAME).hex
	avrdude -p attiny85 -c avrisp -b 19200 -U flash:w:avr.hex:i -F -P /dev/ttyUSB0
