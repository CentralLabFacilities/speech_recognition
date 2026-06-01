# Locate parakeet

# find the include directory
find_path(Parakeet_INCLUDE_DIR parakeet/parakeet.h
        PATH_SUFFIXES include
        PATHS
        /usr/local/include/
        /usr/include/
        ${Parakeet_DIR}/include/)

# find the library
find_library(Parakeet_LIBRARY
        NAMES ${Parakeet_STATIC} parakeet
        PATH_SUFFIXES lib64 lib
        PATHS ~/Library/Frameworks
        /Library/Frameworks
        /usr/local
        /usr
        /sw
        /opt/local
        /opt/csw
        /opt
        ${Parakeet_DIR}/lib)

# handle the QUIETLY and REQUIRED arguments and set YamlCpp_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Parakeet DEFAULT_MSG Parakeet_INCLUDE_DIR Parakeet_LIBRARY)
mark_as_advanced(Parakeet_INCLUDE_DIR Parakeet_LIBRARY)