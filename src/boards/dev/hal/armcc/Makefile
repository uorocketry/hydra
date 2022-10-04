
################################################################################
# Automatically-generated file. Do not edit!
################################################################################

ifdef SystemRoot
	SHELL = cmd.exe
	MK_DIR = mkdir
else
	ifeq ($(shell uname), Linux)
		MK_DIR = mkdir -p
	endif

	ifeq ($(shell uname | cut -d _ -f 1), CYGWIN)
		MK_DIR = mkdir -p
	endif

	ifeq ($(shell uname | cut -d _ -f 1), MINGW32)
		MK_DIR = mkdir -p
	endif

	ifeq ($(shell uname | cut -d _ -f 1), MINGW64)
		MK_DIR = mkdir -p
	endif
endif

# List the subdirectories for creating object files
SUB_DIRS +=  \
 \
hpl/pm \
fatfs/src \
hpl/osc32kctrl \
hpl/can \
hpl/ramecc \
hpl/dmac \
sd_mmc \
hal/src \
hpl/mclk \
hal/utils/src \
examples \
hpl/gclk \
hpl/oscctrl \
hpl/sdhc \
hpl/rtc \
diskio \
hpl/core \
hpl/cmcc

# List the object files
OBJS +=  \
hal/src/hal_io.o \
hal/src/hal_can_async.o \
hal/src/hal_calendar.o \
hpl/can/hpl_can.o \
sd_mmc_start.o \
fatfs_main.o \
hpl/core/hpl_core_m4.o \
hpl/dmac/hpl_dmac.o \
sd_mmc/sd_mmc.o \
hal/src/hal_mci_sync.o \
hal/src/hal_delay.o \
hpl/pm/hpl_pm.o \
hpl/core/hpl_init.o \
hpl/gclk/hpl_gclk.o \
hal/utils/src/utils_list.o \
hal/utils/src/utils_assert.o \
hpl/oscctrl/hpl_oscctrl.o \
hpl/sdhc/hpl_sdhc.o \
hpl/mclk/hpl_mclk.o \
hpl/ramecc/hpl_ramecc.o \
hal/src/hal_init.o \
diskio_start.o \
main.o \
hpl/osc32kctrl/hpl_osc32kctrl.o \
diskio/sdmmc_diskio.o \
hpl/rtc/hpl_rtc.o \
examples/driver_examples.o \
driver_init.o \
hal/src/hal_gpio.o \
hal/utils/src/utils_event.o \
hal/src/hal_sleep.o \
hal/src/hal_cache.o \
hpl/cmcc/hpl_cmcc.o \
atmel_start.o \
fatfs/src/ff.o \
hal/src/hal_atomic.o

OBJS_AS_ARGS +=  \
"hal/src/hal_io.o" \
"hal/src/hal_can_async.o" \
"hal/src/hal_calendar.o" \
"hpl/can/hpl_can.o" \
"sd_mmc_start.o" \
"fatfs_main.o" \
"hpl/core/hpl_core_m4.o" \
"hpl/dmac/hpl_dmac.o" \
"sd_mmc/sd_mmc.o" \
"hal/src/hal_mci_sync.o" \
"hal/src/hal_delay.o" \
"hpl/pm/hpl_pm.o" \
"hpl/core/hpl_init.o" \
"hpl/gclk/hpl_gclk.o" \
"hal/utils/src/utils_list.o" \
"hal/utils/src/utils_assert.o" \
"hpl/oscctrl/hpl_oscctrl.o" \
"hpl/sdhc/hpl_sdhc.o" \
"hpl/mclk/hpl_mclk.o" \
"hpl/ramecc/hpl_ramecc.o" \
"hal/src/hal_init.o" \
"diskio_start.o" \
"main.o" \
"hpl/osc32kctrl/hpl_osc32kctrl.o" \
"diskio/sdmmc_diskio.o" \
"hpl/rtc/hpl_rtc.o" \
"examples/driver_examples.o" \
"driver_init.o" \
"hal/src/hal_gpio.o" \
"hal/utils/src/utils_event.o" \
"hal/src/hal_sleep.o" \
"hal/src/hal_cache.o" \
"hpl/cmcc/hpl_cmcc.o" \
"atmel_start.o" \
"fatfs/src/ff.o" \
"hal/src/hal_atomic.o"

# List the dependency files
DEPS := $(OBJS:%.o=%.d)

DEPS_AS_ARGS +=  \
"hal/src/hal_calendar.d" \
"hal/utils/src/utils_event.d" \
"hal/src/hal_io.d" \
"hpl/ramecc/hpl_ramecc.d" \
"hpl/core/hpl_core_m4.d" \
"fatfs_main.d" \
"hpl/can/hpl_can.d" \
"sd_mmc_start.d" \
"sd_mmc/sd_mmc.d" \
"hal/src/hal_mci_sync.d" \
"hal/utils/src/utils_list.d" \
"hpl/cmcc/hpl_cmcc.d" \
"hpl/dmac/hpl_dmac.d" \
"hal/utils/src/utils_assert.d" \
"hpl/sdhc/hpl_sdhc.d" \
"hal/src/hal_delay.d" \
"hpl/core/hpl_init.d" \
"hpl/pm/hpl_pm.d" \
"hpl/gclk/hpl_gclk.d" \
"diskio_start.d" \
"hal/src/hal_init.d" \
"hpl/mclk/hpl_mclk.d" \
"hal/src/hal_can_async.d" \
"driver_init.d" \
"hpl/osc32kctrl/hpl_osc32kctrl.d" \
"main.d" \
"examples/driver_examples.d" \
"hpl/rtc/hpl_rtc.d" \
"diskio/sdmmc_diskio.d" \
"hal/src/hal_cache.d" \
"hal/src/hal_sleep.d" \
"hal/src/hal_gpio.d" \
"hal/src/hal_atomic.d" \
"fatfs/src/ff.d" \
"hpl/oscctrl/hpl_oscctrl.d" \
"atmel_start.d"

OUTPUT_FILE_NAME :=AtmelStart
QUOTE := "
OUTPUT_FILE_PATH +=$(OUTPUT_FILE_NAME).elf
OUTPUT_FILE_PATH_AS_ARGS +=$(OUTPUT_FILE_NAME).elf

vpath %.c ../
vpath %.s ../
vpath %.S ../

# All Target
all: $(SUB_DIRS) $(OUTPUT_FILE_PATH)

# Linker target

$(OUTPUT_FILE_PATH): $(OBJS)
	@echo Building target: $@
	@echo Invoking: ARMCC Linker
	$(QUOTE)armlink$(QUOTE) --ro-base 0x00000000 --entry 0x00000000 --rw-base 0x20000000 --entry Reset_Handler --first __Vectors \
--strict --summary_stderr --info summarysizes --map --xref --callgraph --symbols \
--info sizes --info totals --info unused --info veneers --list $(OUTPUT_FILE_NAME).map \
-o $(OUTPUT_FILE_NAME).elf --cpu Cortex-M4 \
$(OBJS_AS_ARGS)

	@echo Finished building target: $@

# Compiler target(s)




%.o: %.c
	@echo Building file: $<
	@echo ARMCC Compiler
	$(QUOTE)armcc$(QUOTE) --c99 -c -DDEBUG -O1 -g --apcs=interwork --split_sections --cpu Cortex-M4 -D__SAME51J20A__ \
-I"../" -I"../config" -I"../examples" -I"../hal/include" -I"../hal/utils/include" -I"../hpl/can" -I"../hpl/cmcc" -I"../hpl/core" -I"../hpl/dmac" -I"../hpl/gclk" -I"../hpl/mclk" -I"../hpl/osc32kctrl" -I"../hpl/oscctrl" -I"../hpl/pm" -I"../hpl/port" -I"../hpl/ramecc" -I"../hpl/rtc" -I"../hpl/sdhc" -I"../hri" -I"../" -I"../config" -I"../sd_mmc" -I"../" -I"../diskio" -I"../" -I"../fatfs/src" -I"../" -I"../CMSIS/Core/Include" -I"../include"  \
--depend "$@"  -o "$@" "$<"

	@echo Finished building: $<

%.o: %.s
	@echo Building file: $<
	@echo ARMCC Assembler
	$(QUOTE)armasm$(QUOTE) -g --apcs=interwork --cpu Cortex-M4 --pd "D__SAME51J20A__ SETA 1" \
-I"../" -I"../config" -I"../examples" -I"../hal/include" -I"../hal/utils/include" -I"../hpl/can" -I"../hpl/cmcc" -I"../hpl/core" -I"../hpl/dmac" -I"../hpl/gclk" -I"../hpl/mclk" -I"../hpl/osc32kctrl" -I"../hpl/oscctrl" -I"../hpl/pm" -I"../hpl/port" -I"../hpl/ramecc" -I"../hpl/rtc" -I"../hpl/sdhc" -I"../hri" -I"../" -I"../config" -I"../sd_mmc" -I"../" -I"../diskio" -I"../" -I"../fatfs/src" -I"../" -I"../CMSIS/Core/Include" -I"../include"  \
--depend "$(@:%.o=%.d)" -o "$@" "$<"

	@echo Finished building: $<

%.o: %.S
	@echo Building file: $<
	@echo ARMCC Preprocessing Assembler
	$(QUOTE)armcc$(QUOTE) --c99 -c -DDEBUG -O1 -g --apcs=interwork --split_sections --cpu Cortex-M4 -D__SAME51J20A__ \
-I"../" -I"../config" -I"../examples" -I"../hal/include" -I"../hal/utils/include" -I"../hpl/can" -I"../hpl/cmcc" -I"../hpl/core" -I"../hpl/dmac" -I"../hpl/gclk" -I"../hpl/mclk" -I"../hpl/osc32kctrl" -I"../hpl/oscctrl" -I"../hpl/pm" -I"../hpl/port" -I"../hpl/ramecc" -I"../hpl/rtc" -I"../hpl/sdhc" -I"../hri" -I"../" -I"../config" -I"../sd_mmc" -I"../" -I"../diskio" -I"../" -I"../fatfs/src" -I"../" -I"../CMSIS/Core/Include" -I"../include"  \
--depend "$@"  -o "$@" "$<"

	@echo Finished building: $<

# Detect changes in the dependent files and recompile the respective object files.
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(DEPS)),)
-include $(DEPS)
endif
endif

$(SUB_DIRS):
	$(MK_DIR) "$@"

clean:
	rm -f $(OBJS_AS_ARGS)
	rm -f $(OUTPUT_FILE_PATH)
	rm -f $(DEPS_AS_ARGS)
	rm -f $(OUTPUT_FILE_NAME).map $(OUTPUT_FILE_NAME).elf
