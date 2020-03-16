cmake_minimum_required(VERSION 3.1.0)
project(teknic)

find_package(Threads REQUIRED) # cmake 2.*.* might need it
#generate_messages(DEPENDENCIES std_msgs)
add_compile_options(-std=c++11) #supervisor threads
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(
 include/sFoundationPkg/inc/
 include/sFoundationPkg/inc/inc-pub/
 include/sFoundationPkg/inc/inc-private/linux/
 include/sFoundationPkg/inc/inc-private/sFound/
 include/sFoundationPkg/LibINI/inc/
 include/sFoundationPkg/LibLinuxOS/inc/
 include/sFoundationPkg/LibXML/inc/
 include/sFoundationPkg/inc/inc-private/
 include/
)

add_library(sFoundation
include/sFoundationPkg/sFoundation/src-linux/SerialLinux.cpp
include/sFoundationPkg/sFoundation/src-linux/lnkAccessLinux.cpp
include/sFoundationPkg/sFoundation/src/SerialEx.cpp
include/sFoundationPkg/sFoundation/src/converterLib.cpp
include/sFoundationPkg/sFoundation/src/cpmAPI.cpp
include/sFoundationPkg/sFoundation/src/cpmClassImpl.cpp
include/sFoundationPkg/sFoundation/src/iscAPI.cpp
include/sFoundationPkg/sFoundation/src/lnkAccessCommon.cpp
include/sFoundationPkg/sFoundation/src/meridianNet.cpp
include/sFoundationPkg/sFoundation/src/netCmdAPI.cpp
include/sFoundationPkg/sFoundation/src/netCoreFmt.cpp
include/sFoundationPkg/sFoundation/src/sysClassImpl.cpp
include/sFoundationPkg/LibXML/src/ErrCodeStr.cpp
include/sFoundationPkg/LibXML/src/pugixml.cpp
include/sFoundationPkg/LibLinuxOS/src/tekEventsLinux.cpp
include/sFoundationPkg/LibLinuxOS/src/tekThreadsLinux.cpp
include/sFoundationPkg/LibLinuxOS/src/version.cpp
include/sFoundationPkg/LibINI/src/dictionary.cpp
include/sFoundationPkg/LibINI/src/iniparser.cpp
)
add_executable(trigger src/trigger.cpp )
target_link_libraries(trigger 
    sFoundation
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)
add_executable(main src/Axis.cpp src/Supervisor.cpp src/main.cpp )
target_link_libraries(main 
    sFoundation
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(example src/Axis.cpp src/Supervisor.cpp src/example.cpp )
target_link_libraries(example 
    sFoundation
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(combo src/Servo.cpp src/Brainiac.cpp src/combo.cpp )
target_link_libraries(combo 
    sFoundation
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)


add_executable(SDK_GPIO_Example example/SDK_Examples/GPIO/GPIO.cpp)
target_link_libraries(SDK_GPIO_Example 
    sFoundation
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(SDK_HelloWorld_Example example/SDK_Examples/HelloWorld/HelloWorld.cpp)
target_link_libraries(SDK_HelloWorld_Example 
    sFoundation
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(SDK_Homing_Example example/SDK_Examples/Homing/Homing.cpp)
target_link_libraries(SDK_Homing_Example 
    sFoundation
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(SDK_Motion_Example example/SDK_Examples/Motion/Motion.cpp)
target_link_libraries(SDK_Motion_Example 
    sFoundation
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(SDK_SingleThread_Example example/SDK_Examples/SingleThread/Axis.cpp example/SDK_Examples/SingleThread/SingleThreaded.cpp)
target_link_libraries(SDK_SingleThread_Example 
    sFoundation
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(SDK_MultiThread_Example example/SDK_Examples/MultiThread/Axis.cpp example/SDK_Examples/MultiThread/Supervisor.cpp example/SDK_Examples/MultiThread/MultiThreaded.cpp)
target_link_libraries(SDK_MultiThread_Example 
    sFoundation
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(SDK_StatusAlert_Example example/SDK_Examples/StatusAlerts/StatusAlerts.cpp)
target_link_libraries(SDK_StatusAlert_Example 
    sFoundation
    ${CMAKE_DL_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)


#add_dependencies(clearpath_node clearpath_generate_messages_cpp)
