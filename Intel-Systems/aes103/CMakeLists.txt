## Intel-Systems/aes103 simulator
##
## This is an automagically generated file. Do NOT EDIT.
## Any changes you make will be overwritten!!
##
## Make changes to the SIMH top-level makefile and then run the
## "cmake/generate.py" script to regenerate these files.
##
##     cd cmake; python -m generate --help
##
## ------------------------------------------------------------

if (HAVE_UNITY_FRAMEWORK AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/unit-tests/CMakeLists.txt")
  add_subdirectory(unit-tests)
endif ()

add_simulator(aes103
    SOURCES
        i8080.c
        port.c
        aes103_sys.c
        aes103_irq.c
        aes103_disk.c
        aes103_ram.c
        aes103_mem.c
        aes103_vid.c
        aes103_ioflags.c
        aes103_timer.c
        aes103_kb.c
    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}
    FEATURE_VIDEO
    BUILDROMS
    LABEL Intel-Systems
    PKG_FAMILY ibm_family
    TEST aes103)
