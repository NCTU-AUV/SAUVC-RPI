
if(NOT "/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitinfo.txt" IS_NEWER_THAN "/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src/xrceagent"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src/xrceagent'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout --config "advice.detachedHead=false" "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" "xrceagent"
    WORKING_DIRECTORY "/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/eProsima/Micro-XRCE-DDS-Agent.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout v2.4.2 --
  WORKING_DIRECTORY "/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src/xrceagent"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'v2.4.2'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src/xrceagent"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src/xrceagent'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitinfo.txt"
    "/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/SAUVC2024-RPI/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitclone-lastrun.txt'")
endif()

