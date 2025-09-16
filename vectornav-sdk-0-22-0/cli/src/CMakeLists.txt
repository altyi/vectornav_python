cmake_minimum_required(VERSION 3.16)
add_compile_definitions(NOMINMAX=1)
add_compile_definitions(THREADING_ENABLE=true)
add_compile_definitions(__CLI__=true)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CPP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../cpp)
set(PLUGIN_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../cpp/plugins)
set(CLI_PLUGIN_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../plugins)

add_subdirectory(${CPP_ROOT} oVnSensor)

if(EXISTS "${PLUGIN_ROOT}/Calibration")
	add_subdirectory(${PLUGIN_ROOT}/Calibration CalibrationPlugin)
	set(PLUGIN_CALIBRATION TRUE)
	add_compile_definitions(PLUGIN_CALIBRATION)
endif()

if(EXISTS "${PLUGIN_ROOT}/DataExport")
	add_subdirectory(${PLUGIN_ROOT}/DataExport DataExportPlugin)
	set(PLUGIN_DATAEXPORT TRUE)
	add_compile_definitions(PLUGIN_DATAEXPORT)
endif()

if(EXISTS "${PLUGIN_ROOT}/FirmwareUpdate")
	add_subdirectory(${PLUGIN_ROOT}/FirmwareUpdate FirmwareUpdatePlugin)
	set(PLUGIN_FIRMWAREUPDATE TRUE)
	add_compile_definitions(PLUGIN_FIRMWAREUPDATE)
endif()

if(EXISTS "${PLUGIN_ROOT}/RegisterScan")
	add_subdirectory(${PLUGIN_ROOT}/RegisterScan RegisterScanPlugin)
	set(PLUGIN_REGISTERSCAN TRUE)
	add_compile_definitions(PLUGIN_REGISTERSCAN)
endif()

if(EXISTS "${PLUGIN_ROOT}/SimpleLogger")
	set(PLUGIN_SIMPLELOGGER TRUE)
	add_compile_definitions(PLUGIN_SIMPLELOGGER)
endif()

set(SOURCES
	AssemblyInfo.cpp
	CLI_Sensor.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME} oVnSensor)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_include_directories(${PROJECT_NAME} PRIVATE oVnSensor)

if(PLUGIN_CALIBRATION)
	target_include_directories(${PROJECT_NAME} PRIVATE CalibrationPlugin)
	target_include_directories(${PROJECT_NAME} PUBLIC ${CLI_PLUGIN_ROOT})
	target_link_libraries(${PROJECT_NAME} CalibrationPlugin)
endif()

if(PLUGIN_DATAEXPORT)
	target_include_directories(${PROJECT_NAME} PRIVATE DataExportPlugin)
	target_include_directories(${PROJECT_NAME} PUBLIC ${CLI_PLUGIN_ROOT})
	target_link_libraries(${PROJECT_NAME} DataExportPlugin)
endif()

if(PLUGIN_FIRMWAREUPDATE)
	target_include_directories(${PROJECT_NAME} PRIVATE FirmwareUpdatePlugin)
	target_include_directories(${PROJECT_NAME} PUBLIC ${CLI_PLUGIN_ROOT})
	target_link_libraries(${PROJECT_NAME} FirmwareUpdatePlugin)
endif()

if(PLUGIN_REGISTERSCAN)
	target_include_directories(${PROJECT_NAME} PRIVATE RegisterScanPlugin)
	target_include_directories(${PROJECT_NAME} PUBLIC ${CLI_PLUGIN_ROOT})
	target_link_libraries(${PROJECT_NAME} RegisterScanPlugin)
endif()

if(PLUGIN_SIMPLELOGGER)
	target_include_directories(${PROJECT_NAME} PRIVATE ${PLUGIN_ROOT}/SimpleLogger)
	target_include_directories(${PROJECT_NAME} PUBLIC ${CLI_PLUGIN_ROOT})
endif()


SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "/clr")
STRING(REPLACE "/EHsc" "/EHa" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
STRING(REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG})

# Copy output dll to "net" directory
set(DLL_OUTPUT_DIR ${CPP_ROOT}/../net/)

if(NOT EXISTS "${DLL_OUTPUT_DIR}")
	file(MAKE_DIRECTORY "${DLL_OUTPUT_DIR}")
endif()

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:VnSdk_Net> ${DLL_OUTPUT_DIR})
