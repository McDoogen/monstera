add_executable(motor_controller
    motor_controller.c
    pico_uart_transport.c
)

target_link_libraries(motor_controller pico_stdlib hardware_pwm microros) #TODO:DS: What is microros here?
target_include_directories(motor_controller PUBLIC ${micro_ros_SOURCE_DIR}/libmicroros/include)
#TODO:DS: Can I add the pico-sdk directory here too? Instead of in c_cpp_properties?

# Configure Pico
pico_enable_stdio_usb(motor_controller 1)
pico_enable_stdio_uart(motor_controller 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(motor_controller)

# TODO:DS: How do I include the UF2 in the install destination?
install(TARGETS motor_controller DESTINATION .)