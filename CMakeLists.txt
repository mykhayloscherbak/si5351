cmake_minimum_required(VERSION 3.25)
project(ksi C)

set(CMAKE_C_STANDARD 11)
set(MCU attiny10)

add_compile_definitions(NDEBUG)
add_compile_options(-x c -c -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall  -std=gnu99 -ffreestanding  -mmcu=${MCU})
add_link_options(-Os -flto -fuse-linker-plugin -mmcu=${MCU} -Wl,--gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_executable(ksi main.c)


set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${HEX_FILE}
        COMMENT "Building ${HEX_FILE}")

add_custom_target(flash
        ${AVR_DUDE_EXECUTABLE} -c  avrispmkII -p ${MCU}  ${FLASH_VERBOSE_FLAG} -e -qq -B 100khz -U flash:w:${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex:a
        DEPENDS ${PROJECT_NAME}
        COMMENT "Flash to ${MCU}")