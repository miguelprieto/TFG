cmake_minimum_required(VERSION 2.6)
project(xbee-tools)

# Cerca libreria LZO
find_path(LZO_INCLUDE_DIR lzo/lzo1x.h)
find_library(LZO_LIBRARIES NAMES lzo2)

if (NOT (LZO_INCLUDE_DIR AND LZO_LIBRARIES))
	message(SEND_ERROR "La libreria LZO non è stata trovata")
endif()

# Cerca libreria pthread
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)
if (CMAKE_USE_PTHREADS_INIT)
	set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")
endif()

include_directories(${LZO_INCLUDE_DIR})
link_libraries(${LZO_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

set(xbeemux_SRCS
	# Core
	xbeemux-core/main.cpp
	xbeemux-core/MessageQueue.cpp
	xbeemux-core/XBee.cpp
	xbeemux-core/XBeeAddress.cpp
	xbeemux-core/XBeeDriver.cpp
	xbeemux-core/Service.cpp
	xbeemux-core/TcpServer.cpp
	xbeemux-core/Utils.cpp

	# Servizio xbeetap
	xbeetap/XBeeTapClientHandler.cpp
	xbeetap/XBeeTapService.cpp

	# Servizio xbeertsp
	xbeertsp/DeviceInfo.cpp
	xbeertsp/HexImage.cpp
	xbeertsp/NotifyStatus.cpp
	xbeertsp/Utils.cpp
	xbeertsp/XBeeRtspClientHandler.cpp
	xbeertsp/XBeeRtspEndpointInfo.cpp
	xbeertsp/XBeeRtspInterface.cpp
	xbeertsp/XBeeRtspJobManager.cpp
	xbeertsp/XBeeRtspJobs.cpp
	xbeertsp/XBeeRtspRequestParser.cpp
	xbeertsp/XBeeRtspService.cpp

	# Servizio xbeestdio
	xbeestdio/XBeeStdioClientHandler.cpp
	xbeestdio/XBeeStdioEndpoint.cpp
	xbeestdio/XBeeStdioMessage.cpp
	xbeestdio/XBeeStdioService.cpp
)

add_executable(xbeemux ${xbeemux_SRCS})
install(TARGETS xbeemux DESTINATION sbin)

configure_file(xbeemux.service.in ${CMAKE_CURRENT_BINARY_DIR}/xbeemux.service @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/xbeemux.service DESTINATION lib/systemd/system)

set(xbeedump_SRCS
	xbeemux-core/XBee.cpp
	xbeemux-core/XBeeAddress.cpp
	xbeemux-core/XBeeDriver.cpp
	xbeemux-core/Utils.cpp
	xbeetap/xbeedump.cpp
)

add_executable(xbeedump ${xbeedump_SRCS})
install(TARGETS xbeedump DESTINATION sbin)

set(rtspcmd_SRCS
	xbeemux-core/Utils.cpp
	xbeertsp/rtspcmd.cpp
	xbeertsp/DeviceInfo.cpp
	xbeertsp/HexImage.cpp
	xbeertsp/NotifyStatus.cpp
	xbeertsp/Utils.cpp
)

add_executable(rtspcmd ${rtspcmd_SRCS})
install(TARGETS rtspcmd DESTINATION bin)

set(hexinfo_SRCS
	xbeertsp/hexinfo.cpp
	xbeertsp/DeviceInfo.cpp
	xbeertsp/HexImage.cpp
	xbeertsp/Utils.cpp
)

add_executable(hexinfo ${hexinfo_SRCS})
install(TARGETS hexinfo DESTINATION bin)
