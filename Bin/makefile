TARGET = firmware
SRC_DIR = ../Src
SRC = $(wildcard $(SRC_DIR)/*.c)
OBJ = $(notdir $(SRC:.c=.o))
LST = $(OBJ:.o=.lst)
LIB_DIR = ../Src/Lib
LIB_SRC = $(wildcard $(LIB_DIR)/*.c)
LIB_OBJ = $(notdir $(LIB_SRC:.c=.o))


DEVICE = atmega16a
SPEED = 14745600

OBJCOPY = avr-objcopy
AVRSIZE = avr-size
AVROBJDUMP = avr-objdump
AR = avr-ar

ARCH = -mmcu=${DEVICE}
SPEED = -DF_CPU=${SPEED}UL

CC = avr-gcc
CFLAGS += -Os
CFLAGS += -Wfatal-errors
CFLAGS += -Wall
CFLAGS += -funsigned-char
CFLAGS += -funsigned-bitfields
CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -Wl,--gc-sections
CFLAGS += -std=gnu99
CFLAGS += -ffunction-sections 
CFLAGS += -fdata-sections
#CFLAGS += -fdiagnostics-color=always -MMD -MP -MF $(@F).d

LDFLAGS = -Wl,--gc-sections 
LCFLAGS += -Wl,-u,vfprintf -lprintf_min


PROGRAMMER = USBasp
AVRDUDE = avrdude
AVRDUDE_FLAGS = -c ${PROGRAMMER} -p ${DEVICE}

DEL=del

all: build size

build: $(TARGET).hex
eep: $(TARGET).eep
debug: $(TARGET).lst


#%.a: $(LIB_SRC)

lib: $(LIB_DIR)/./*.c
	${CC} ${CFLAGS} -c $< -o test.o

%.o: $(SRC_DIR)/./%.c
	@echo
	@echo ######   Compiling: $<   ######   
	${CC} ${CFLAGS} ${ARCH} ${SPEED} -c $< -o $@ 2>"compilation_output_$(notdir $($<:.c=)).txt"

.SECONDARY: $(TARGET).elf
.PRECIOUS: $(OBJ)
$(TARGET).elf: $(OBJ)
	@echo
	@echo ######   Linking: $^   ######   
	${CC} ${CFLAGS} ${LDFLAGS} ${ARCH} $^ -o $@

%.hex: %.elf
	@echo
	@echo ######   Converting $< to $@   ######   
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

size: $(TARGET).elf
	@echo
	${AVRSIZE} --mcu=${DEVICE} -C -x $<  
	${AVRSIZE} -B -x $< --mcu=${DEVICE} -d

%.eep: $(TARGET).elf
	@echo
	@echo ######   Getting EEPROM image: $@   ######   
	${OBJCOPY} -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex $< $@

clean:
	@echo
	@echo ######   Cleaning up   ######   
	-${DEL} "*.elf" 
	-${DEL} "*.hex" 
	-${DEL} "*.lst" 
	-${DEL} "*.o" 
	-${DEL} "*.eep" 
	-${DEL} "compilation_output*"

%.lst: $(TARGET).elf
	@echo
	@echo ######   Creating listing file: $@ for debug   ######   
	${AVROBJDUMP} -h -S -s $< > $@

install: ${TARGET}.hex
	@echo
	@echo ######   Programming device: $(DEVICE) with: $<   ######   
	${AVRDUDE} ${AVRDUDE_FLAGS} -U flash:w:${DIR}/$<

program_eeprom:	${TARGET}.eep
	@echo
	@echo ######   Writing $< to $(DEVICE) EEPROM   ######   
	${AVRDUDE} ${AVRDUDE_FLAGS} -U eeprom:w:${DIR}/$<


.DEFAULTGOAL: all
.PHONY: all build eep debug install program_eeprom clean