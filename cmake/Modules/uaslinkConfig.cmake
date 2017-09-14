INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_UASLINK uaslink)

FIND_PATH(
    UASLINK_INCLUDE_DIRS
    NAMES uaslink/api.h
    HINTS $ENV{UASLINK_DIR}/include
        ${PC_UASLINK_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    UASLINK_LIBRARIES
    NAMES gnuradio-uaslink
    HINTS $ENV{UASLINK_DIR}/lib
        ${PC_UASLINK_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(UASLINK DEFAULT_MSG UASLINK_LIBRARIES UASLINK_INCLUDE_DIRS)
MARK_AS_ADVANCED(UASLINK_LIBRARIES UASLINK_INCLUDE_DIRS)

