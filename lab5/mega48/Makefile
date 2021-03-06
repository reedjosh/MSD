PRG             =main

OBJS            =$(PRG).o uart_functions_m48.o ../libs/lm73_functions.o  ../libs/twi_master.o  

SRCS            =$(PRG).c  uart_functions_m48.c ../libs/lm73_functions.c ../libs/twi_master.c

MCU_TARGET     = atmega48

PROGRAMMER_TARGET     = m48

OPTIMIZE       = -O2    # options are 1, 2, 3, s

F_CPU          = 8000000UL

DEFS           =
LIBS           =

CC             = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -DF_CPU=$(F_CPU)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text #eeprom

$(PRG).elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean: 
	find . -name '*.d' -delete 
	find . -name '*.elf' -delete  
	find . -name '*.lst' -delete 
	find . -name '*.map' -delete 
	find . -name '*.bin' -delete
	find . -name '*.hex' -delete
	find . -name '*.srec' -delete
	find . -name '*.o' -delete

# setup for usb programmer
program: $(PRG).hex
	avrdude -p $(PROGRAMMER_TARGET) -c osuisp2 -p m48 -e -U flash:w:$(PRG).hex 

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

