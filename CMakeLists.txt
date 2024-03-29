cmake_minimum_required(VERSION 3.9 FATAL_ERROR)
   project(elliptecpp VERSION 0.1 DESCRIPTION "C++ Thorlabs elliptec library")
   include(ExternalProject)
   set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
   set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
   set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
   
   set(default_build_type "Release")
      if(CMAKE_BUILD_TYPE STREQUAL "")
         message(STATUS "Build type not specified. defaulting to '${default_build_type}")
         set(CMAKE_BUILD_TYPE "${default_build_type}")
      endif()
   if(CMAKE_BUILD_TYPE STREQUAL "Debug")
      message(STATUS "Debug build")
      add_compile_options("-fsanitize=address")
      add_compile_options("-fsanitize=leak")
      add_compile_options("-fsanitize=undefined")
      add_compile_options("-fno-sanitize-recover=all")
      add_compile_options("-fsanitize=float-divide-by-zero")
      add_compile_options("-fno-sanitize=null")
      add_compile_options("-fno-sanitize=alignment")
      add_compile_options("-fsanitize=float-cast-overflow")
      add_compile_options("-fsanitize-address-use-after-scope")
      add_compile_options("-fstack-protector-all")
      add_compile_options("-fharden-compares")
      add_compile_options("-fharden-conditional-branches")
      add_compile_options("-fstack-check")
      link_libraries("-fsanitize=address -fsanitize=leak -fsanitize=undefined -fsanitize-address-use-after-scope -fstack-protector-all -fharden-compares -fharden-conditional-branches -fstack-check -fno-sanitize-recover=all -fno-sanitize=alignment -fno-sanitize=null -fsanitize=float-divide-by-zero -fsanitize=float-cast-overflow")
   endif()
   
   if(NOT CMAKE_DEBUG_POSTFIX)
      set(CMAKE_DEBUG_POSTFIX d)
   endif()
   if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(CMAKE_C_FLAGS "-Wall -Wextra -fPIC ")
      set(CMAKE_C_FLAGS_DEBUG "-O0 -g -ggdb -fPIC -z nodelete")
      set(CMAKE_C_FLAGS_RELEASE "-Werror -pipe -g -O3 -fPIC -march=native -fomit-frame-pointer -fno-signaling-nans -fno-trapping-math -ftree-vectorize")
      set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC ")
      set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb -fPIC -z nodelete")
      set(CMAKE_CXX_FLAGS_RELEASE "-Werror -pipe -g -O3 -fPIC -march=native -fomit-frame-pointer -fno-signaling-nans -fno-trapping-math -ftree-vectorize ")
   elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
      set(CMAKE_C_FLAGS "-Wall -Wextra -fPIC")
      set(CMAKE_C_FLAGS_DEBUG "-O0 -g -ggdb -fPIC")
      set(CMAKE_C_FLAGS_RELEASE "-Werror -pipe -g -O3 -fPIC -march=native -ipo -fomit-frame-pointer -fno-signaling-nans -fno-trapping-math -ftree-vectorize")
      set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
      set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb -fPIC")
      set(CMAKE_CXX_FLAGS_RELEASE "-Werror -pipe -g -O3 -fPIC -march=native -ipo -fomit-frame-pointer -fno-signaling-nans -fno-trapping-math -ftree-vectorize")
   endif()
   
   
   find_package(Boost 1.70 REQUIRED COMPONENTS system OPTIONAL_COMPONENTS program_options)
   find_package(Git REQUIRED)
   
   option(BUILD_EXAMPLES "Build all example programs" ON)
   if (BUILD_EXAMPLES)
      add_subdirectory(src/examples)
   endif(BUILD_EXAMPLES)

   
   add_library(elliptecpp SHARED src/ell.cpp src/ell_util.cpp src/ell_comm.cpp src/boost_serial.cpp)

   set_target_properties(elliptecpp PROPERTIES VERSION ${PROJECT_VERSION})
   set_target_properties(elliptecpp PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})
   set_target_properties(elliptecpp PROPERTIES PUBLIC_HEADER include/elliptec.h)
   set_target_properties(elliptecpp PROPERTIES PUBLIC_HEADER include/elliptec.h)
   
   set_target_properties(elliptecpp PROPERTIES 
                                    CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
                                    CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
                                    CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
   
   target_include_directories(elliptecpp PRIVATE include)
   target_include_directories(elliptecpp PRIVATE src)
   
   include(GNUInstallDirs)
   
   install(TARGETS elliptecpp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    configure_file(elliptecpp.pc.in elliptecpp.pc @ONLY)
   
   set_property(TARGET elliptecpp PROPERTY CXX_STANDARD 20)
   
   install(FILES ${CMAKE_BINARY_DIR}/elliptecpp.pc DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
   
   target_link_libraries(elliptecpp ${Boost_LIBRARIES})
   
   if (CMAKE_BUILD_TYPE STREQUAL "Release")
      include(CheckIPOSupported)
      check_ipo_supported(RESULT hasipo OUTPUT error)
      if( hasipo )
         message(STATUS "IPO / LTO enabled")
         set_property(TARGET elliptecpp PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
      else()
         message(STATUS "IPO / LTO not supported: <${error}>")
      endif()
   endif()
