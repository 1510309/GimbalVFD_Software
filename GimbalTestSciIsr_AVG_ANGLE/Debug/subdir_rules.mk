################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
F2806x_CodeStartBranch.obj: C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/source/F2806x_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define=_DEBUG --define=LARGE_MODEL --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="F2806x_CodeStartBranch.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2806x_CpuTimers.obj: C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/source/F2806x_CpuTimers.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define=_DEBUG --define=LARGE_MODEL --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="F2806x_CpuTimers.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2806x_DefaultIsr.obj: C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/source/F2806x_DefaultIsr.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define=_DEBUG --define=LARGE_MODEL --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="F2806x_DefaultIsr.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2806x_ECap.obj: C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/source/F2806x_ECap.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define=_DEBUG --define=LARGE_MODEL --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="F2806x_ECap.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2806x_EPwm.obj: C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/source/F2806x_EPwm.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define=_DEBUG --define=LARGE_MODEL --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="F2806x_EPwm.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2806x_GlobalVariableDefs.obj: C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/source/F2806x_GlobalVariableDefs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define=_DEBUG --define=LARGE_MODEL --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="F2806x_GlobalVariableDefs.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2806x_PieCtrl.obj: C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/source/F2806x_PieCtrl.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define=_DEBUG --define=LARGE_MODEL --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="F2806x_PieCtrl.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2806x_PieVect.obj: C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/source/F2806x_PieVect.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define=_DEBUG --define=LARGE_MODEL --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="F2806x_PieVect.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2806x_SysCtrl.obj: C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/source/F2806x_SysCtrl.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define=_DEBUG --define=LARGE_MODEL --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="F2806x_SysCtrl.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2806x_usDelay.obj: C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/source/F2806x_usDelay.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define=_DEBUG --define=LARGE_MODEL --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="F2806x_usDelay.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

GimbalTest.obj: ../GimbalTest.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_headers/include" --include_path="C:/ti/controlSUITE/device_support/f2806x/v151/F2806x_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" -g --define=_DEBUG --define=LARGE_MODEL --quiet --verbose_diagnostics --diag_warning=225 --diag_suppress=10063 --issue_remarks --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="GimbalTest.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


