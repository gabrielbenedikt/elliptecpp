set(CPPLINENOISE_INSTALL_LOCATION ${CMAKE_BINARY_DIR})
   ExternalProject_Add(cpplinenoise
   PREFIX ${CMAKE_BINARY_DIR}/ext
    GIT_REPOSITORY https://github.com/yhirose/cpp-linenoise
    UPDATE_COMMAND ${GIT_EXECUTABLE} pull
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    )
include_directories(${CMAKE_BINARY_DIR}/ext/src/cpplinenoise)
include_directories(${CMAKE_SOURCE_DIR}/include)
message(${CMAKE_BINARY_DIR}/ext/src/cpplinenoise)

if (boost_program_options_FOUND)
   message(STATUS "Example ell_interactive will be built")
   add_executable(ell_interactive ell_interactive.cpp)
   add_dependencies(ell_interactive cpplinenoise)
   set_property(TARGET ell_interactive PROPERTY CXX_STANDARD 20)
   target_link_libraries(ell_interactive ${Boost_LIBRARIES} elliptecpp)
   target_include_directories(ell_interactive PUBLIC ${Boost_INCLUDE_DIR})
else()
   message(STATUS "Boost program_options missing. Example ell_interactive will not be built.")
endif(boost_program_options_FOUND)

add_executable(ell_rot ell_rot.cpp)
set_property(TARGET ell_rot PROPERTY CXX_STANDARD 20)
target_link_libraries(ell_rot ${Boost_LIBRARIES} elliptecpp)
target_include_directories(ell_rot PUBLIC ${Boost_INCLUDE_DIR})
                                    
include(CheckIPOSupported)
check_ipo_supported(RESULT hasipo OUTPUT error)
if( hasipo )
    message(STATUS "IPO / LTO enabled")
    set_property(TARGET ell_rot PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    if (boost_program_options_FOUND)
       set_property(TARGET ell_interactive PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif(boost_program_options_FOUND)
else()
    message(STATUS "IPO / LTO not supported: <${error}>")
endif()
