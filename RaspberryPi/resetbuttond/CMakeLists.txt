cmake_minimum_required(VERSION 2.6)
project(resetbuttond NONE)

install(PROGRAMS resetbuttond DESTINATION sbin)

configure_file(resetbuttond.service.in ${CMAKE_CURRENT_BINARY_DIR}/resetbuttond.service @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/resetbuttond.service DESTINATION lib/systemd/system)
