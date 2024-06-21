add_executable(motor_controller
    motor_controller.c
)

target_link_libraries(motor_controller pico_stdlib hardware_pwm)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(motor_controller)

# TODO:DS: How do I include the UF2 in the install destination?
install(TARGETS motor_controller DESTINATION .)