cmake_minimum_required(VERSION 2.8.12)
project(Assignment1)

### Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

### Compilation flags: adapt to your needs ###
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /bigobj") ### Enable parallel compilation
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR} )
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR} )
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") #### Libigl requires a modern C++ compiler that supports c++11
  set(TBB_LINK_FLAGS "-ltbb")
  set(TBBROOT "/usr/local/Cellar/tbb/2018_U5")

  ### set(TBBLIB_DEBUG "")
endif()

### Add src to the include directories
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")
LINK_DIRECTORIES(${LINK_DIRECTORIES} /opt/intel/compilers_and_libraries_2019.0.117/mac/tbb/lib)

find_library(TBB_LIB tbb)
if (TBB_LIB)
  message('Found_Intel_tbb_library.')
  SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${TBB_LINK_FLAGS}")
endif()

### Include Eigen for linear algebra
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ext/eigen")
### include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ext/tbb")
### Compile all the cpp files in src
file(GLOB SOURCES
"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_executable(${PROJECT_NAME}_bin ${SOURCES})
