add_executable(hello
    hello.c
)

target_link_libraries(hello pico_stdlib hardware_pwm)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(hello)

# TODO:DS: How do I include the UF2 in the install destination?
install(TARGETS hello DESTINATION .)