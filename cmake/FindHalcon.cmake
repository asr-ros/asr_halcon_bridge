find_path(HALCON_INCLUDE_DIR NAMES Halcon.h HINTS $ENV{HALCONROOT}/include)
find_library(HALCON_LIBRARY NAMES halcon HINTS $ENV{HALCONROOT}/lib/$ENV{HALCONARCH})
find_library(HALCONCPP_LIBRARY NAMES halconcpp HINTS $ENV{HALCONROOT}/lib/$ENV{HALCONARCH})

set(Halcon_LIBRARIES ${HALCON_LIBRARY} ${HALCONCPP_LIBRARY})
set(Halcon_INCLUDE_DIRS ${HALCON_INCLUDE_DIR} ${HALCON_INCLUDE_DIR}/halconcpp)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Halcon DEFAULT_MSG
                                  Halcon_LIBRARIES Halcon_INCLUDE_DIRS)

mark_as_advanced(Halcon_INCLUDE_DIRS Halcon_LIBRARIES)