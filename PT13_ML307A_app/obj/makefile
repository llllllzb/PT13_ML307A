################################################################################
# MRS Version: 1.9.0
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Task/subdir.mk
-include StdPeriphDriver/subdir.mk
-include Startup/subdir.mk
-include RVMSIS/subdir.mk
-include HAL/subdir.mk
-include APP/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
PT13_app.hexAndbin \

SECONDARY_LIST += \
PT13_app.lst \

SECONDARY_SIZE += \
PT13_app.siz \


# All Target
all: PT13_app.elf secondary-outputs

# Tool invocations
PT13_app.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -mcmodel=medany -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -T "../Ld/Link.ld" -nostartfiles -Xlinker --gc-sections -L"../" -L"R:\CODE\CHcode\CH582\PT13_ML307A\PT13_ML307A_app\LIB" -L"../StdPeriphDriver" -Xlinker --print-memory-usage -Wl,-Map,"PT13_app.map" --specs=nano.specs -u _printf_float --specs=nosys.specs -o "PT13_app.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
PT13_app.hexAndbin: PT13_app.elf
	@	riscv-none-embed-objcopy -O ihex "PT13_app.elf"  "PT13_app.hex" && riscv-none-embed-objcopy -O binary "PT13_app.elf"  "PT13_app.bin"
	@	@
PT13_app.lst: PT13_app.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble "PT13_app.elf" > "PT13_app.lst"
	@	@
PT13_app.siz: PT13_app.elf
	@	riscv-none-embed-size --format=berkeley "PT13_app.elf"
	@	@
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) PT13_app.elf
	-@
post-build:
	-riscv-none-embed-objcopy -O ihex "PT13_app.elf" "PT13_app.hex" && riscv-none-embed-objcopy -O binary "PT13_app.elf" "PT13_app.bin"
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents post-build

-include ../makefile.targets
