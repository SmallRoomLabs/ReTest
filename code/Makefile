PRG            	= ReTest
PORT			=/dev/ttyUSB0

MCU_TARGET     	= atmega328p
OPTIMIZE       	= -O2
DEFS           	= -D F_CPU=16000000
LIBS           	=

CC             	= avr-gcc
OBJCOPY        	= avr-objcopy
OBJDUMP        	= avr-objdump
INFO			= avr-size
FLASH			= avrdude

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map

all: $(PRG).elf lst $(PRG).hex

$(PRG).elf: $(PRG).o spi.o lcd.o rotaryenc.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

# dependencies:
$(PRG).o: $(PRG).c
spi.o: spi.c spi.h
lcd.o: lcd.c lcd.h
rotaryenc.o: rotaryenc.c rotaryenc.h

clean:
	rm -rf *.o *.elf *.eps *.png *.pdf *.bak
	rm -rf *.lst *.map *.hex *.bin *.srec *~

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
	$(INFO) --format=avr --mcu=$(MCU_TARGET) $(PRG).elf

upload:
	$(FLASH) -q -q -p$(MCU_TARGET) -carduino -P$(PORT) -b57600 -D -Uflash:w:$(PRG).hex:i

