add_executable(
  fun_blaster
  main.c
  ws.c
  ws.S
  ircodes.c
  tx.c
  )

target_link_libraries(
  fun_blaster
  pico_switch
  pico_task
  pico_stdlib
  pico_util
  pico_divider
  pico_multicore
  hardware_adc
  hardware_dma
  hardware_pwm
  hardware_timer
  )

pico_add_extra_outputs(fun_blaster)
pico_enable_stdio_usb(fun_blaster ENABLED)

set_property(TARGET fun_blaster PROPERTY C_STANDARD 23)
target_compile_definitions(fun_blaster PUBLIC PICO_MAX_SHARED_IRQ_HANDLERS=16u)
target_compile_options(fun_blaster PRIVATE -Wall -Wextra -Wnull-dereference)

#pico_set_binary_type(fun_blaster no_flash)
#pico_set_binary_type(fun_blaster copy_to_ram)

# vim:set sw=2 ts=2 et:
