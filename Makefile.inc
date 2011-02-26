# GCC settings
export CC=avr-gcc
export OBJCOPY=avr-objcopy
export OBJCOPYFLAGS=
export MCU=attiny2313
# Speed of the CPU oscillator
export F_CPU =8000000
# Speed of the XTAL oscillator
export XTAL =8000000
export CFLAGS =-Os -I.. -DF_CPU=$(F_CPU) -DXTAL=$(XTAL) -mmcu=$(MCU)

# avrdude settings
export DUDE=avrdude
export DUDE_MCU=t2313
export DUDE_PROGRAMMER=usbtiny
export DUDE_FLAGS =-c $(DUDE_PROGRAMMER) -p $(DUDE_MCU) -e 

