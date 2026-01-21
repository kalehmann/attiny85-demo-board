AVRDUDE = avrdude
AVRDUDE_FLAGS = -c usbtiny -p attiny85
CC = avr-gcc
CC_FLAGS = -DF_CPU=1000000 -mmcu=attiny85
CC_EXTRA_FLAGS = -Wall -Wextra -Werror
OBJCOPY = avr-objcopy


BUILD_DIR = build
DEMO_HEX = $(BUILD_DIR)/demo.hex

.PHONY: all build clean flash

all: build

build: $(DEMO_HEX)

$(BUILD_DIR)/%.elf: $(BUILD_DIR)/%.o
	$(CC) $(CC_FLAGS) -o $@ $<

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/%.o: src/%.c
	@mkdir -p $(@D)
	$(CC) -c -Os $(CC_FLAGS) $(CC_EXTRA_FLAGS) -o $@ $<

flash: build
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$(DEMO_HEX)

clean:
	rm -rf $(BUILD_DIR)

