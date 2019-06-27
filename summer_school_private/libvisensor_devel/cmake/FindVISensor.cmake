# - Try to find ImageMagick++
# Once done, this will define
#
#  Magick++_FOUND - system has Magick++
#  Magick++_INCLUDE_DIRS - the Magick++ include directories
#  Magick++_LIBRARIES - link these to use Magick++

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(VISensorDriver_PKGCONF visensor)

# Include dir
find_path(VISensorDriver_INCLUDE_DIR
  NAMES aslam_sensor_driver.hpp
  PATHS ${VISensorDriver_PKGCONF_INCLUDE_DIRS}
)


# Finally the library itself
find_library(VISensorDriver_LIBRARY
  NAMES libvisensor.so
  PATHS ${VISensorDriver_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(VISensorDriver_PROCESS_INCLUDES VISensorDriver_INCLUDE_DIR)
set(VISensorDriver_PROCESS_LIBS VISensorDriver_LIBRARY)
libfind_process(VISensorDriver)
