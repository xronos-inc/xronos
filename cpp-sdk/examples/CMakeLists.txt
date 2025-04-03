add_executable(hello_world hello_world.cc)
target_link_libraries(hello_world xronos-sdk)

add_executable(timers timers.cc)
target_link_libraries(timers xronos-sdk)

add_executable(counter counter.cc)
target_link_libraries(counter xronos-sdk)

add_executable(reflex_game reflex_game.cc)
target_link_libraries(reflex_game xronos-sdk)

add_executable(metrics metrics.cc)
target_link_libraries(metrics xronos-sdk)
