################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
BSP/src/%.o: ../BSP/src/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-armllvm_4.0.0.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/Board" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/BSP" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/BSP/inc" -I"D:/CCS/workspace_v12/MYCode_v6_MPUMode/Debug" -I"D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -MMD -MP -MF"BSP/src/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


