# Compile the firmware for Renode: the same sources and the same flags as the CubeIDE Debug build
# (-O2 -g3 -fno-strict-aliasing, newlib-nano with float printf), minus two hardware seams Renode
# cannot model. No CubeIDE needed.
#
#   make master SETUP=<setup>                -> build/<setup>/master.elf
#   make slave  SETUP=<setup> SLAVE=slaveN   -> build/<setup>/slaveN.elf
#
# Use `python3 robosim.py build` rather than calling make directly: each slave ELF must be compiled
# against its own generated motor_config.h, and robosim.py regenerates that header per slave (and
# restores the tree's generated files afterwards). The generated headers are inputs here.
#
# The seams (see README.md "What is real and what is substituted"):
#   master: shim/master_host_uart.c replaces the ST USB stack + usbd_cdc_if.c (no USB device model)
#   slave:  shim/slave_spi_dma.c replaces HAL_SPI_TransmitReceive_DMA / HAL_SPI_Abort_IT only
#           (the HAL's own copies are renamed inside stm32f4xx_hal_spi.c)

ROOT    := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
SIM     := $(ROOT)/sim/renode
SETUP   ?= default
SLAVE   ?= slave0
OUT     := $(SIM)/build/$(SETUP)
CC      := arm-none-eabi-gcc
SIZE    := arm-none-eabi-size

CPU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS  := $(CPU) -std=gnu11 -g3 -O2 -fno-strict-aliasing -ffunction-sections -fdata-sections -Wall \
           -DSTM32F446xx -DUSE_HAL_DRIVER -DDEBUG $(EXTRA_CFLAGS)
LDFLAGS  = $(CPU) -static --specs=nano.specs -u _printf_float -Wl,--gc-sections \
           -Wl,-Map=$(basename $@).map -Wl,--start-group -lc -lm -Wl,--end-group

.PHONY: master slave clean
master: $(OUT)/master.elf
slave: $(OUT)/$(SLAVE).elf

# -- master --------------------------------------------------------------------
M       := $(ROOT)/firmware/master
M_SRC   := $(wildcard $(M)/Core/Src/*.c) $(wildcard $(M)/Drivers/STM32F4xx_HAL_Driver/Src/*.c) \
           $(SIM)/shim/master_host_uart.c
M_ASM   := $(M)/Core/Startup/startup_stm32f446retx.s
M_INC   := -I$(M)/Core/Inc -I$(M)/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I$(M)/Drivers/CMSIS/Include \
           -I$(M)/Drivers/STM32F4xx_HAL_Driver/Inc -I$(M)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
           -I$(M)/USB_DEVICE/App -I$(M)/USB_DEVICE/Target \
           -I$(M)/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
           -I$(M)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I$(ROOT)/firmware/common/include
M_OBJ   := $(patsubst $(ROOT)/%.c,$(OUT)/master/%.o,$(M_SRC)) $(patsubst $(ROOT)/%.s,$(OUT)/master/%.o,$(M_ASM))

$(OUT)/master/%.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(M_INC) -MMD -MP -c $< -o $@
$(OUT)/master/%.o: $(ROOT)/%.s
	@mkdir -p $(dir $@)
	$(CC) $(CPU) -x assembler-with-cpp -c $< -o $@
$(OUT)/master.elf: $(M_OBJ) $(M)/STM32F446RETX_FLASH.ld
	$(CC) $(M_OBJ) $(LDFLAGS) -T$(M)/STM32F446RETX_FLASH.ld -o $@
	$(SIZE) $@

# -- slave (one ELF per slave of the setup) ------------------------------------
S       := $(ROOT)/firmware/slave/slave_general
S_SRC   := $(wildcard $(S)/Core/Src/*.c) $(wildcard $(S)/Drivers/STM32F4xx_HAL_Driver/Src/*.c) \
           $(SIM)/shim/slave_spi_dma.c
S_ASM   := $(S)/Core/Startup/startup_stm32f446retx.s
S_INC   := -I$(S)/Core/Inc -I$(S)/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I$(S)/Drivers/CMSIS/Include \
           -I$(S)/Drivers/STM32F4xx_HAL_Driver/Inc -I$(S)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
           -I$(ROOT)/firmware/common/include
# CubeIDE also passes -D__FPU_PRESENT=1 -D__FPU_USED=1; stm32f446xx.h and core_cm4.h already define
# both to 1 for this part with -mfloat-abi=hard, so they are left out here (same code, no redefinition noise).
S_DEF   :=
S_OBJ   := $(patsubst $(ROOT)/%.c,$(OUT)/$(SLAVE)/%.o,$(S_SRC)) $(patsubst $(ROOT)/%.s,$(OUT)/$(SLAVE)/%.o,$(S_ASM))

$(OUT)/$(SLAVE)/firmware/slave/slave_general/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.o: \
    S_SEAM := -DHAL_SPI_TransmitReceive_DMA=HAL_SPI_TransmitReceive_DMA_hal -DHAL_SPI_Abort_IT=HAL_SPI_Abort_IT_hal

$(OUT)/$(SLAVE)/%.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(S_DEF) $(S_SEAM) $(S_INC) -MMD -MP -c $< -o $@
$(OUT)/$(SLAVE)/%.o: $(ROOT)/%.s
	@mkdir -p $(dir $@)
	$(CC) $(CPU) -x assembler-with-cpp -c $< -o $@
$(OUT)/$(SLAVE).elf: $(S_OBJ) $(S)/STM32F446RETX_FLASH.ld
	$(CC) $(S_OBJ) $(LDFLAGS) -T$(S)/STM32F446RETX_FLASH.ld -o $@
	$(SIZE) $@

clean:
	rm -rf $(SIM)/build

-include $(M_OBJ:.o=.d) $(S_OBJ:.o=.d)
