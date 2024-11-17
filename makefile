# TODO makefile cleanup
OUTPUT_DIR=Bin

TARGET = firmware
SRC = $(wildcard *.c)
OBJ = $(SRC:.c=.o)
HDR = $(SRC:.c=.h)
#LST = $(SRC:.c=.lst)


DEVICE=atmega16a
SPEED=14745600

OBJCOPY=avr-objcopy
AVRSIZE=avr-size
AVROBJDUMP=avr-objdump

ARCH = -mmcu=${DEVICE}

CC=avr-gcc
CFLAGS += -DF_CPU=${SPEED}UL
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

LDFLAGS = -Wl,--gc-sections 
# -Wl,--start-group -Wl,-lm  -Wl,--end-group
# LDFLAGS += 

PROGRAMMER = USBasp
AVRDUDE = avrdude
AVRDUDE_FLAGS = -c ${PROGRAMMER} -p ${DEVICE}

DEL=del

all: build size

build: $(TARGET).hex

%.o: %.c $(HDR)
	${CC} ${CFLAGS} ${ARCH} -c $< -o $@ 2>"${OUTPUT_DIR}/compilation_output_$<.txt"

$(TARGET).elf: $(OBJ)
	${CC} ${LDFLAGS} ${ARCH} $^ -o $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

size: ${TARGET}.elf
	${AVRSIZE} --mcu=${DEVICE} -C -x $<
	${AVRSIZE} -B -x $< --mcu=${DEVICE} -d

clean:
	${DEL} "*.o" "*.elf" "*.hex"

# all: build size

# build: elf hex

# elf: ${TARGET}.elf
# hex: ${TARGET}.elf

# %.hex: %.elf
# 		$(OBJCOPY) -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures -O ihex $< $@

# .SECONDARY : $(TARGET).elf
# .PRECIOUS : $(OBJ)

# %.elf: $(OBJ)
# 		$(CC) -mmcu=$(DEVICE) $(LDFLAGS) $^ -o $@

# ${OUTPUT_DIR}/%.o: %.c
# 		${CC} ${CFLAGS} -c $< -o $@ 2>${OUTPUT_DIR}/compilation_output.txt
# #		$(CC) $(CFLAGS) -c $< -o $(@F) 2>${DIR}/compilation_output$(@F).txt


# all: hex install

# hex: ${TARGET}.elf

# %.elf: %.c
# 		${CC} ${CFLAGS} $< -o ${DIR}/$@ 2>${DIR}/compilation_output.txt

# %.hex: %.elf
# 		${OBJCOPY} -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures -O ihex ${DIR}/$< ${DIR}/$@
# 		${AVRSIZE} --mcu=${DEVICE} -C -x ${DIR}/$<
# 		${AVRSIZE} -B -x ${DIR}/$< --mcu=${DEVICE} -d

# eeprom: ${BIN}.elf
# 		${OBJCOPY} -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex ${DIR}/$< ${DIR}/${TARGET}.eep

# debug:
# 		${AVROBJDUMP} -h -Ss ${DIR}/${TARGET}.elf > ${DIR}/${TARGET}.lst

# install: ${TARGET}.hex
# 		${AVRDUDE} ${AVRDUDE_FLAGS} -U flash:w:${DIR}/$<

# program_eeprom:	${TARGET}.eep
# 		${AVRDUDE} ${AVRDUDE_FLAGS} -U eeprom:w:${DIR}/$<

# clean:
# 	${DEL} "${DIR}\${TARGET}.elf" "${DIR}\${TARGET}.hex" "${DIR}\${TARGET}.lst" "${DIR}\${OBJS}" "${DIR}\${TARGET}.eep" "${DIR}\compilation_output.txt"


# .DEFAULTGOAL: all
# .PHONY: all hex eeprom debug install program_eeprom clean

# avrdude -c USBasp -p atmega16a	-U lfuse:w:0xFE:m	-U hfuse:w:0xC1:m