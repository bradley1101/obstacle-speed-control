# make            build obstacle.hex
# make flash      upload with avrdude (Arduino Uno bootloader)
# make size       memory usage
MCU     = atmega328p
TARGET  = obstacle
PORT   ?= /dev/ttyACM0
CC      = avr-gcc
CFLAGS  = -mmcu=$(MCU) -Os -std=c11 -Wall -Wextra -Werror

all: $(TARGET).hex

$(TARGET).elf: main.c config.h
	$(CC) $(CFLAGS) -o $@ main.c

$(TARGET).hex: $(TARGET).elf
	avr-objcopy -O ihex -R .eeprom $< $@

size: $(TARGET).elf
	avr-size --mcu=$(MCU) -C $<

flash: $(TARGET).hex
	avrdude -p $(MCU) -c arduino -P $(PORT) -b 115200 -U flash:w:$<:i

clean:
	rm -f $(TARGET).elf $(TARGET).hex

.PHONY: all size flash clean
