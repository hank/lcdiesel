include ../Makefile.inc
CFLAGS += -I../include
TARGETS=$(patsubst %.c, %.hex, $(wildcard *.c))
OBJS=$(patsubst %.c, %.o, $(wildcard *.c))

%.o: %.c ../lib/lcd.c
	$(CC) $(CFLAGS) -o $@ $< ../lib/lcd.o

%.hex: %.o
	$(OBJCOPY) $(OBJCOPYFLAGS) -R .eeprom -R .fuse -R .lock -O ihex $< $@

all: $(TARGETS)

clean:
	rm -f $(TARGETS) $(OBJS)

flash_test_custom_characters:
	sudo $(DUDE) $(DUDE_FLAGS) -U flash:w:test_custom_characters.hex

flash_test_lcd_puts:
	sudo $(DUDE) $(DUDE_FLAGS) -U flash:w:test_lcd_puts.hex
