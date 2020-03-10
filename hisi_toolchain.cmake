set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(HISI_ROOT "/opt/hisi-linux/x86-arm/aarch64-himix100-linux")

set(CMAKE_C_COMPILER "${HISI_ROOT}/bin/aarch64-himix100-linux-gcc")
set(CMAKE_CXX_COMPILER "${HISI_ROOT}/bin/aarch64-himix100-linux-g++")

set(CMAKE_SYSROOT ${HISI_ROOT}/target)



set(CMAKE_FIND_ROOT_PATH
    $ENV{ROS2_INSTALL_PATH}
)

# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -lddsc -lyaml")

MESSAGE(STATUS ${CMAKE_FIND_ROOT_PATH})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# This assumes that pthread will be available on the target system
    # (this emulates that the return of the TRY_RUN is a return code "0"
set(THREADS_PTHREAD_ARG "0"
      CACHE STRING "Result from TRY_RUN" FORCE)