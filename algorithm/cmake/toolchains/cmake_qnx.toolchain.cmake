set(CMAKE_SYSTEM_NAME QNX)
set(CMAKE_CMAKE_SYSTEM_VERSION 7.0.0)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(arch aarch64le)
## QNX BASE
if(DEFINED ENV{QNX_BASE})
  set(QNX_BASE $ENV{QNX_BASE})
  message(STATUS "Found QNX_BASE = ${QNX_BASE}")
else()
  message(FATAL_ERROR "QNX_BASE was not found")
endif()

set(ENV{QNX_HOST} ${QNX_BASE}/host/linux/x86_64)
set(ENV{QNX_TARGET} ${QNX_BASE}/target/qnx7)

set(QNX_HOST $ENV{QNX_HOST})
set(QNX_TARGET $ENV{QNX_TARGET})

set(CFLAGS "-DNV_SDK_BUILD \
            -O3 \
            -fno-strict-aliasing \
            -fno-common \
            -fomit-frame-pointer \
            -finline-functions \
            -fpic \
            -ftree-vectorize \
            -fstack-protector-strong \
            -Wcast-align \
            -DWIN_INTERFACE_CUSTOM \
            -D_FILE_OFFSET_BITS=64 \
            -D_QNX_SOURCE \
            -D_POSIX_C_SOURCE=200112L \
            -fno-builtin-memcpy \
            -finline-limit=300")
set(CXXFLAGS "-DNV_SDK_BUILD \
              -O3 \
              -fno-strict-aliasing \
              -fno-common \
              -fomit-frame-pointer \
              -finline-functions \
              -fpic \
              -ftree-vectorize \
              -fstack-protector-strong \
              -Wcast-align \
              -DWIN_INTERFACE_CUSTOM \
              -D_FILE_OFFSET_BITS=64 \
              -D_QNX_SOURCE \
              -D_POSIX_C_SOURCE=200112L \
              -fno-builtin-memcpy \
              -finline-limit=300")

set(CMAKE_SIZEOF_VOID_P 64)

set(CMAKE_C_FLAGS ${CFLAGS} CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS ${CXXFLAGS}  CACHE STRING "" FORCE)

set(CMAKE_C_COMPILER ${QNX_HOST}/usr/bin/aarch64-unknown-nto-qnx7.0.0-gcc)
set(CMAKE_CXX_COMPILER ${QNX_HOST}/usr/bin/aarch64-unknown-nto-qnx7.0.0-g++)

set(CMAKE_C_COMPILER_TARGET ${arch})
set(CMAKE_CXX_COMPILER_TARGET ${arch})

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

set(CMAKE_SYSROOT ${QNX_TARGET})

set(CMAKE_CUDA_COMPILER_FORCED TRUE)
set(CMAKE_CUDA_FLAGS "-m64 -ccbin ${CMAKE_CXX_COMPILER}")
