include(FindPackageHandleStandardArgs)

find_path (Halcon_INCLUDE_DIR halconcpp/HalconCpp.h
  PATHS
  $ENV{HALCON_ROOT}/include
  /usr/local/include
  /usr/include
  /sw/include
  /opt/halcon/include
)

find_library (Halcon_ENGINE_LIBRARY hdevenginecpp
  PATHS
  $ENV{HALCON_ROOT}/lib/x64-linux
  /usr/local/lib/x64-linux
  /usr/lib/x64-linux
  /lib/x64-linux
  /sw/lib/x64-linux
  /opt/halcon/lib/x64-linux
)

find_library (Halcon_CPP_LIBRARY halconcpp
  PATHS
  $ENV{HALCON_ROOT}/lib/x64-linux
  /usr/local/lib/x64-linux
  /usr/lib/x64-linux
  /lib/x64-linux
  /sw/lib/x64-linux
  /opt/halcon/lib/x64-linux
)

find_library (Halcon_LIBRARY halcon
  PATHS
  $ENV{HALCON_ROOT}/lib/x64-linux
  /usr/local/lib/x64-linux
  /usr/lib/x64-linux
  /lib/x64-linux
  /sw/lib/x64-linux
  /opt/halcon/lib/x64-linux
)


find_package_handle_standard_args(Halcon
  DEFAULT_MSG
  Halcon_INCLUDE_DIR
  Halcon_LIBRARY
)

if(Halcon_FOUND)
  set(Halcon_LIBRARIES ${Halcon_LIBRARY} ${Halcon_ENGINE_LIBRARY} ${Halcon_CPP_LIBRARY})
  set(Halcon_INCLUDE_DIRS ${Halcon_INCLUDE_DIR})
endif()

mark_as_advanced(Halcon_INCLUDE_DIRS
  Halcon_LIBRARIES
  )
