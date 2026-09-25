#
# J30D sensor driver
#
# Consumable three ways:
#
#   1. Vendored:   add_subdirectory(J30D_API)   then link j30d::j30d
#   2. Installed:  find_package(j30d)           then link j30d::j30d
#   3. Not at all: compile src/*.c + config/j30d_config.c with inc/ on the
#                  include path. Nothing here is required -- see README.md.
#
# The driver core is portable C99 with no dependencies. Everything platform
# specific lives behind the function pointers in j30d_platform.h, so this
# file never needs to know about your HAL, RTOS or toolchain.
#
cmake_minimum_required(VERSION 3.15)

# Single source of truth for the version is j30d_version.h -- parse it rather
# than repeating it here, so the two can never drift.
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/inc/j30d_version.h" _j30d_ver_lines
     REGEX "^#define[ \t]+J30D_API_VERSION_(MAJOR|MINOR|PATCH)[ \t]+[0-9]+")
foreach(_part MAJOR MINOR PATCH)
    foreach(_line IN LISTS _j30d_ver_lines)
        if(_line MATCHES "J30D_API_VERSION_${_part}[ \t]+([0-9]+)")
            set(_j30d_${_part} "${CMAKE_MATCH_1}")
        endif()
    endforeach()
endforeach()
if(NOT DEFINED _j30d_MAJOR OR NOT DEFINED _j30d_MINOR OR NOT DEFINED _j30d_PATCH)
    message(FATAL_ERROR "Could not parse J30D_API_VERSION_* from inc/j30d_version.h")
endif()

project(j30d
        VERSION ${_j30d_MAJOR}.${_j30d_MINOR}.${_j30d_PATCH}
        DESCRIPTION "Firmware driver for the Jondetech J30D / JIRS30D sensor"
        LANGUAGES C)

# Are we the top-level project, or vendored into someone else's build?
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(_j30d_standalone TRUE)
else()
    set(_j30d_standalone FALSE)
endif()

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------

# Which reference port to compile in, if any. The default is "none": the
# ports are reference implementations that #include your project's headers
# (main.h, i2c.h, the STM32 HAL, FreeRTOS), so they cannot build standalone.
# Most integrations copy the closest port into their own tree and adjust it.
set(J30D_PORT "none" CACHE STRING
    "Reference platform port to compile: none | stm32-baremetal | stm32-freertos")
set_property(CACHE J30D_PORT PROPERTY STRINGS none stm32-baremetal stm32-freertos)

option(J30D_USE_BUNDLED_CONFIG
       "Compile config/j30d_config.c (the compiled-in j30d_default_config). \
Turn OFF to supply your own definition of j30d_default_config." ON)

option(J30D_NO_FLOAT
       "Drop every float from the API: j30d_field_t loses .value and \
j30d_calib_t carries raw fixed-point words. For FPU-less parts." OFF)

# Destructive-write guards. All three protect against states the driver
# cannot talk the device out of; see the notes in j30d.h. Leave them OFF
# unless you specifically need the capability (production tooling, or a
# product that really does run the sensor in PWM/Relay mode).
option(J30D_ALLOW_NON_I2C_OUTPUT
       "Permit writes that move SYS_CFG1's OUT_MODE off I2C. This is \
EEPROM-backed and survives power cycles -- it can leave a unit unreachable." OFF)
option(J30D_ALLOW_EEPROM_BLOW
       "Permit writes to BLOW_START (0x40). Blows the EEPROM fuse, irreversible." OFF)
option(J30D_ALLOW_CALIBRATION_WRITE
       "Permit writes to the factory calibration block (0xC0..0xEF). \
Destroys the unit's calibration permanently." OFF)

# Init handshake timing. Empty means "leave the driver's own default".
set(J30D_RESET_SETTLE_DELAY_MS "" CACHE STRING
    "Override the post-soft-reset settle delay in ms (driver default: 50)")
set(J30D_EEPROM_POLL_ATTEMPTS "" CACHE STRING
    "Override the EEPROM-loaded poll attempt count (driver default: 5)")
set(J30D_EEPROM_POLL_DELAY_MS "" CACHE STRING
    "Override the delay between EEPROM-loaded polls in ms (driver default: 10)")

option(J30D_BUILD_TESTS "Build the host-side test suite" ${_j30d_standalone})
option(J30D_WERROR "Treat compiler warnings as errors" OFF)
option(J30D_INSTALL "Generate install and package-config rules" ${_j30d_standalone})

# ---------------------------------------------------------------------------
# The library
# ---------------------------------------------------------------------------

add_library(j30d STATIC
    src/j30d.c
    src/j30d_version.c
)
add_library(j30d::j30d ALIAS j30d)

if(J30D_USE_BUNDLED_CONFIG)
    target_sources(j30d PRIVATE config/j30d_config.c)
endif()

if(NOT J30D_PORT STREQUAL "none")
    if(J30D_PORT STREQUAL "stm32-baremetal")
        target_sources(j30d PRIVATE ports/platform_stm32_baremetal.c)
    elseif(J30D_PORT STREQUAL "stm32-freertos")
        target_sources(j30d PRIVATE ports/platform_stm32_freertos.c)
    else()
        message(FATAL_ERROR
            "J30D_PORT='${J30D_PORT}' is not a known port. "
            "Use none, stm32-baremetal or stm32-freertos. "
            "(ports/platform_zephyr_mockup.c is an uncompiled sketch, not a port -- see its header.)")
    endif()
    target_include_directories(j30d PRIVATE ports)
    message(STATUS
        "j30d: compiling port '${J30D_PORT}'. It #includes your project's headers "
        "(main.h, i2c.h, the vendor HAL, and FreeRTOS for the RTOS port) -- add those "
        "include directories and link the HAL, or this target will not compile.")
endif()

target_include_directories(j30d
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
        $<INSTALL_INTERFACE:include>
)

# The driver is C99. Prefer the feature request (it propagates to consumers),
# but some cross toolchains ship without a compile-feature table and would
# hard-error on it -- fall back to the plain standard property there rather
# than failing the configure.
if("c_std_99" IN_LIST CMAKE_C_COMPILE_FEATURES)
    target_compile_features(j30d PUBLIC c_std_99)
else()
    set_target_properties(j30d PROPERTIES C_STANDARD 99)
endif()

# These change public struct layouts and API behaviour, so they must reach
# every translation unit that includes the headers -- hence PUBLIC.
foreach(_opt J30D_NO_FLOAT
             J30D_ALLOW_NON_I2C_OUTPUT
             J30D_ALLOW_EEPROM_BLOW
             J30D_ALLOW_CALIBRATION_WRITE)
    if(${_opt})
        target_compile_definitions(j30d PUBLIC ${_opt})
    endif()
endforeach()

# Timing overrides only affect j30d.c, so PRIVATE is enough.
foreach(_tunable J30D_RESET_SETTLE_DELAY_MS
                 J30D_EEPROM_POLL_ATTEMPTS
                 J30D_EEPROM_POLL_DELAY_MS)
    if(NOT "${${_tunable}}" STREQUAL "")
        target_compile_definitions(j30d PRIVATE ${_tunable}=${${_tunable}}u)
    endif()
endforeach()

# ---------------------------------------------------------------------------
# Warnings
#
# Applied to this project's own targets only (PRIVATE), never pushed onto
# consumers. The set below is what the driver is kept clean against.
# ---------------------------------------------------------------------------

set(J30D_WARNING_FLAGS "")

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    set(J30D_WARNING_FLAGS
        -Wall -Wextra -Wpedantic
        -Wconversion -Wsign-conversion
        -Wshadow -Wcast-qual -Wdouble-promotion
        -Wstrict-prototypes -Wmissing-prototypes
    )
    if(J30D_WERROR)
        list(APPEND J30D_WARNING_FLAGS -Werror)
    endif()
elseif(MSVC)
    set(J30D_WARNING_FLAGS /W4)
    if(J30D_WERROR)
        list(APPEND J30D_WARNING_FLAGS /WX)
    endif()
endif()

# PRIVATE: these are how we keep OUR sources clean; they are never imposed on
# a consumer. Kept as a plain list rather than an INTERFACE target so that
# install(EXPORT) has nothing extra to resolve.
target_compile_options(j30d PRIVATE ${J30D_WARNING_FLAGS})

# ---------------------------------------------------------------------------
# Tests
#
# fake_sensor_test.c drives the real driver against an in-memory fake sensor
# that models the awkward parts of the hardware (sleep cutting off I2C, the
# status registers clearing all their DRDY bits on read). It is pure host C
# with no hardware, so it runs anywhere.
#
# The build options above change struct layouts and refusal behaviour, so the
# suite is built once per interesting combination rather than once overall --
# a NO_FLOAT or guards-disabled build that nobody compiles is a build that
# quietly rots.
# ---------------------------------------------------------------------------

if(J30D_BUILD_TESTS)
    enable_testing()

    # One executable per interesting option combination. Written out
    # explicitly rather than packed into strings: the option names would
    # otherwise have to be separated by ";", which is CMake's own list
    # separator, and the entries silently split apart.
    function(j30d_add_test_variant name)
        set(_tgt fake_sensor_test_${name})
        add_executable(${_tgt}
            test/fake_sensor_test.c
            src/j30d.c src/j30d_version.c config/j30d_config.c)
        target_include_directories(${_tgt} PRIVATE inc)
        target_compile_options(${_tgt} PRIVATE ${J30D_WARNING_FLAGS})
        if(ARGN)
            target_compile_definitions(${_tgt} PRIVATE ${ARGN})
        endif()
        add_test(NAME fake_sensor_${name} COMMAND ${_tgt})
    endfunction()

    j30d_add_test_variant(default)
    j30d_add_test_variant(nofloat   J30D_NO_FLOAT)
    j30d_add_test_variant(unguarded J30D_ALLOW_NON_I2C_OUTPUT
                                    J30D_ALLOW_EEPROM_BLOW
                                    J30D_ALLOW_CALIBRATION_WRITE)
    j30d_add_test_variant(nofloat_unguarded
                                    J30D_NO_FLOAT
                                    J30D_ALLOW_NON_I2C_OUTPUT
                                    J30D_ALLOW_EEPROM_BLOW
                                    J30D_ALLOW_CALIBRATION_WRITE)
endif()

# ---------------------------------------------------------------------------
# Install / package config
# ---------------------------------------------------------------------------

if(J30D_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    install(TARGETS j30d EXPORT j30dTargets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(DIRECTORY inc/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
            FILES_MATCHING PATTERN "*.h")
    install(EXPORT j30dTargets
            FILE j30dTargets.cmake
            NAMESPACE j30d::
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/j30d)

    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/j30dConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMinorVersion)   # no ABI guarantee -- see j30d_version.h

    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/j30dConfig.cmake"
"include(\"\${CMAKE_CURRENT_LIST_DIR}/j30dTargets.cmake\")\n")

    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/j30dConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/j30dConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/j30d)
endif()

if(_j30d_standalone)
    message(STATUS "j30d ${PROJECT_VERSION}: port=${J30D_PORT} no_float=${J30D_NO_FLOAT} tests=${J30D_BUILD_TESTS}")
endif()
