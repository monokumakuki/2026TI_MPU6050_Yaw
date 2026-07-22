################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-armllvm_4.0.0.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/Board" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/BSP" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/BSP/inc" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/Debug" -I"D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1803757871: ../empty.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"D:/ti/CCS_SDK/sysconfig_1.26.2/sysconfig_cli.bat" --script "D:/CCS/workspace_v12/MYCode_v6_MPUMode/empty.syscfg" -o "." -s "D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1803757871 ../empty.syscfg
device.opt: build-1803757871
device.cmd.genlibs: build-1803757871
ti_msp_dl_config.c: build-1803757871
ti_msp_dl_config.h: build-1803757871
Event.dot: build-1803757871

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-armllvm_4.0.0.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/Board" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/BSP" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/BSP/inc" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/Debug" -I"D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-armllvm_4.0.0.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/Board" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/BSP" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/BSP/inc" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/Debug" -I"D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


