# ##############################################################################
# #################       THIRD PARTY LIBRARIES          ######################
# #################    Copyright (c) 2026 Hello World    ######################
# ##############################################################################

# ---- CMSIS-DSP（ahrs / BMI088 依赖的数学库） -------------------------------
set(CMSIS_DSP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dsp/CMSIS-DSP")

file(GLOB_RECURSE cmsis_dsp_srcs "${CMSIS_DSP_DIR}/Source/*.c")

add_library(cmsis_dsp STATIC ${cmsis_dsp_srcs})
target_include_directories(
  cmsis_dsp
  PUBLIC ${CMSIS_DSP_DIR}/Include
  PRIVATE ${CMSIS_DSP_DIR}/PrivateInclude)
target_compile_definitions(cmsis_dsp
  PUBLIC ARM_MATH_MATRIX_CHECK ARM_MATH_ROUNDING)

# CMSIS-DSP 依赖 Cortex-M 内核头（cmsis_compiler.h / core_cm4.h），
# 它们随 Drivers 一起提供
target_link_libraries(cmsis_dsp PUBLIC drivers)
