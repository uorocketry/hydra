
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
hpl/ramecc \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/RVDS/ARM_CM4F \
examples \
hpl/systick \
hpl/oscctrl \
hpl/sdhc \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0 \
fatfs/src \
hpl/osc32kctrl \
hpl/dmac \
hal/src \
hal/utils/src \
diskio \
hpl/rtc \
hpl/can \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/MemMang \
hpl/pm \
hpl/cmcc \
hpl/gclk \
hpl/mclk \
hpl/pac \
sd_mmc \
hpl/sercom \
hpl/core \
hpl/dsu

# List the object files
OBJS +=  \
hal/src/hal_io.o \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/list.o \
hpl/systick/hpl_systick.o \
hal/src/hal_can_async.o \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/event_groups.o \
hal/src/hal_calendar.o \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/croutine.o \
hpl/can/hpl_can.o \
sd_mmc_start.o \
fatfs_main.o \
hpl/core/hpl_core_m4.o \
hal/src/hal_cache.o \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/stream_buffer.o \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/MemMang/heap_1.o \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/queue.o \
hal/src/hal_mci_sync.o \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/timers.o \
hal/src/hal_delay.o \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/RVDS/ARM_CM4F/port.o \
hpl/pm/hpl_pm.o \
hpl/core/hpl_init.o \
hpl/gclk/hpl_gclk.o \
hal/utils/src/utils_list.o \
hal/utils/src/utils_assert.o \
hpl/pac/hpl_pac.o \
hpl/sdhc/hpl_sdhc.o \
hal/src/hal_usart_sync.o \
hpl/mclk/hpl_mclk.o \
hpl/ramecc/hpl_ramecc.o \
hal/src/hal_init.o \
diskio_start.o \
hal/src/hal_crc_sync.o \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/rtos_port.o \
rtos_start.o \
hpl/osc32kctrl/hpl_osc32kctrl.o \
diskio/sdmmc_diskio.o \
hpl/rtc/hpl_rtc.o \
examples/driver_examples.o \
hpl/oscctrl/hpl_oscctrl.o \
driver_init.o \
hpl/sercom/hpl_sercom.o \
hal/src/hal_atomic.o \
main.o \
hal/src/hal_gpio.o \
hal/utils/src/utils_event.o \
hal/src/hal_sleep.o \
hpl/dmac/hpl_dmac.o \
sd_mmc/sd_mmc.o \
atmel_start.o \
fatfs/src/ff.o \
hpl/cmcc/hpl_cmcc.o \
thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/tasks.o \
hpl/dsu/hpl_dsu.o

OBJS_AS_ARGS +=  \
"hal/src/hal_io.o" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/list.o" \
"hpl/systick/hpl_systick.o" \
"hal/src/hal_can_async.o" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/event_groups.o" \
"hal/src/hal_calendar.o" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/croutine.o" \
"hpl/can/hpl_can.o" \
"sd_mmc_start.o" \
"fatfs_main.o" \
"hpl/core/hpl_core_m4.o" \
"hal/src/hal_cache.o" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/stream_buffer.o" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/MemMang/heap_1.o" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/queue.o" \
"hal/src/hal_mci_sync.o" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/timers.o" \
"hal/src/hal_delay.o" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/RVDS/ARM_CM4F/port.o" \
"hpl/pm/hpl_pm.o" \
"hpl/core/hpl_init.o" \
"hpl/gclk/hpl_gclk.o" \
"hal/utils/src/utils_list.o" \
"hal/utils/src/utils_assert.o" \
"hpl/pac/hpl_pac.o" \
"hpl/sdhc/hpl_sdhc.o" \
"hal/src/hal_usart_sync.o" \
"hpl/mclk/hpl_mclk.o" \
"hpl/ramecc/hpl_ramecc.o" \
"hal/src/hal_init.o" \
"diskio_start.o" \
"hal/src/hal_crc_sync.o" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/rtos_port.o" \
"rtos_start.o" \
"hpl/osc32kctrl/hpl_osc32kctrl.o" \
"diskio/sdmmc_diskio.o" \
"hpl/rtc/hpl_rtc.o" \
"examples/driver_examples.o" \
"hpl/oscctrl/hpl_oscctrl.o" \
"driver_init.o" \
"hpl/sercom/hpl_sercom.o" \
"hal/src/hal_atomic.o" \
"main.o" \
"hal/src/hal_gpio.o" \
"hal/utils/src/utils_event.o" \
"hal/src/hal_sleep.o" \
"hpl/dmac/hpl_dmac.o" \
"sd_mmc/sd_mmc.o" \
"atmel_start.o" \
"fatfs/src/ff.o" \
"hpl/cmcc/hpl_cmcc.o" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/tasks.o" \
"hpl/dsu/hpl_dsu.o"

# List the dependency files
DEPS := $(OBJS:%.o=%.d)

DEPS_AS_ARGS +=  \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/event_groups.d" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/croutine.d" \
"hal/src/hal_calendar.d" \
"hal/utils/src/utils_event.d" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/stream_buffer.d" \
"hal/src/hal_io.d" \
"hpl/ramecc/hpl_ramecc.d" \
"hpl/systick/hpl_systick.d" \
"hpl/core/hpl_core_m4.d" \
"fatfs_main.d" \
"diskio/sdmmc_diskio.d" \
"hpl/can/hpl_can.d" \
"sd_mmc_start.d" \
"hal/src/hal_crc_sync.d" \
"main.d" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/timers.d" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/rtos_port.d" \
"sd_mmc/sd_mmc.d" \
"hal/src/hal_mci_sync.d" \
"hal/utils/src/utils_list.d" \
"hpl/cmcc/hpl_cmcc.d" \
"hpl/pac/hpl_pac.d" \
"hal/utils/src/utils_assert.d" \
"hpl/sdhc/hpl_sdhc.d" \
"hal/src/hal_delay.d" \
"hpl/core/hpl_init.d" \
"hpl/pm/hpl_pm.d" \
"hpl/gclk/hpl_gclk.d" \
"hpl/dmac/hpl_dmac.d" \
"diskio_start.d" \
"hal/src/hal_init.d" \
"hal/src/hal_usart_sync.d" \
"hpl/mclk/hpl_mclk.d" \
"hal/src/hal_can_async.d" \
"driver_init.d" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/MemMang/heap_1.d" \
"hpl/osc32kctrl/hpl_osc32kctrl.d" \
"rtos_start.d" \
"examples/driver_examples.d" \
"hpl/rtc/hpl_rtc.d" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/RVDS/ARM_CM4F/port.d" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/list.d" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/queue.d" \
"hal/src/hal_cache.d" \
"hal/src/hal_sleep.d" \
"hpl/sercom/hpl_sercom.d" \
"hal/src/hal_gpio.d" \
"hal/src/hal_atomic.d" \
"fatfs/src/ff.d" \
"hpl/oscctrl/hpl_oscctrl.d" \
"hpl/dsu/hpl_dsu.d" \
"thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/tasks.d" \
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
-I"../" -I"../config" -I"../examples" -I"../hal/include" -I"../hal/utils/include" -I"../hpl/can" -I"../hpl/cmcc" -I"../hpl/core" -I"../hpl/dmac" -I"../hpl/dsu" -I"../hpl/gclk" -I"../hpl/mclk" -I"../hpl/osc32kctrl" -I"../hpl/oscctrl" -I"../hpl/pac" -I"../hpl/pm" -I"../hpl/port" -I"../hpl/ramecc" -I"../hpl/rtc" -I"../hpl/sdhc" -I"../hpl/sercom" -I"../hpl/systick" -I"../hri" -I"../" -I"../config" -I"../thirdparty/RTOS" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/include" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/RVDS/ARM_CM4F" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/module_config" -I"../" -I"../diskio" -I"../" -I"../fatfs/src" -I"../" -I"../config" -I"../sd_mmc" -I"../" -I"../CMSIS/Core/Include" -I"../include"  \
--depend "$@"  -o "$@" "$<"

	@echo Finished building: $<

%.o: %.s
	@echo Building file: $<
	@echo ARMCC Assembler
	$(QUOTE)armasm$(QUOTE) -g --apcs=interwork --cpu Cortex-M4 --pd "D__SAME51J20A__ SETA 1" \
-I"../" -I"../config" -I"../examples" -I"../hal/include" -I"../hal/utils/include" -I"../hpl/can" -I"../hpl/cmcc" -I"../hpl/core" -I"../hpl/dmac" -I"../hpl/dsu" -I"../hpl/gclk" -I"../hpl/mclk" -I"../hpl/osc32kctrl" -I"../hpl/oscctrl" -I"../hpl/pac" -I"../hpl/pm" -I"../hpl/port" -I"../hpl/ramecc" -I"../hpl/rtc" -I"../hpl/sdhc" -I"../hpl/sercom" -I"../hpl/systick" -I"../hri" -I"../" -I"../config" -I"../thirdparty/RTOS" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/include" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/RVDS/ARM_CM4F" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/module_config" -I"../" -I"../diskio" -I"../" -I"../fatfs/src" -I"../" -I"../config" -I"../sd_mmc" -I"../" -I"../CMSIS/Core/Include" -I"../include"  \
--depend "$(@:%.o=%.d)" -o "$@" "$<"

	@echo Finished building: $<

%.o: %.S
	@echo Building file: $<
	@echo ARMCC Preprocessing Assembler
	$(QUOTE)armcc$(QUOTE) --c99 -c -DDEBUG -O1 -g --apcs=interwork --split_sections --cpu Cortex-M4 -D__SAME51J20A__ \
-I"../" -I"../config" -I"../examples" -I"../hal/include" -I"../hal/utils/include" -I"../hpl/can" -I"../hpl/cmcc" -I"../hpl/core" -I"../hpl/dmac" -I"../hpl/dsu" -I"../hpl/gclk" -I"../hpl/mclk" -I"../hpl/osc32kctrl" -I"../hpl/oscctrl" -I"../hpl/pac" -I"../hpl/pm" -I"../hpl/port" -I"../hpl/ramecc" -I"../hpl/rtc" -I"../hpl/sdhc" -I"../hpl/sercom" -I"../hpl/systick" -I"../hri" -I"../" -I"../config" -I"../thirdparty/RTOS" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/include" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/Source/portable/RVDS/ARM_CM4F" -I"../thirdparty/RTOS/freertos/FreeRTOSV10.0.0/module_config" -I"../" -I"../diskio" -I"../" -I"../fatfs/src" -I"../" -I"../config" -I"../sd_mmc" -I"../" -I"../CMSIS/Core/Include" -I"../include"  \
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
