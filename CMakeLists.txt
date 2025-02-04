# Get the latest Git tag
execute_process(
    COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE LIB_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

idf_component_register(SRCS "ads1115_adc.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "driver")

# Add the version definition to the build
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY COMPILE_DEFINITIONS LIB_VERSION=\"${LIB_VERSION}\")
