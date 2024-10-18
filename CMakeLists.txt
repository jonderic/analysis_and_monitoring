add_executable(analysis_and_monitoring
analysis_and_monitoring.c
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(analysis_and_monitoring pico_stdlib hardware_pwm hardware_adc hardware_timer)

# create map/bin/hex file etc.
pico_add_extra_outputs(analysis_and_monitoring)

# enable usb for i/o
pico_enable_stdio_usb(analysis_and_monitoring 1)

# add url via pico_set_program_url
example_auto_set_url(analysis_and_monitoring)
