cmake_minimum_required(VERSION 3.25)
project(orbit_simulation
        VERSION 1.0.0
        DESCRIPTION "A physical orbit simulation using raylib"
        LANGUAGES CXX)

# Enable C++23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable warnings and modern CMake behaviors
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE OFF)



# Including external dependencies
include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

# Configure Raylib
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY "https://github.com/raysan5/raylib.git"
    GIT_TAG "master"
    GIT_PROGRESS TRUE
)

# Configure Eigen
FetchContent_Declare(
    eigen
    GIT_REPOSITORY "https://gitlab.com/libeigen/eigen.git"
    GIT_TAG "3.4.0"
    GIT_PROGRESS TRUE
)

# Make dependencies available
FetchContent_MakeAvailable(raylib eigen)

# Create target
add_executable(${PROJECT_NAME})

# Find source files
file(GLOB_RECURSE PROJECT_SOURCES
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# Set target sources
target_sources(${PROJECT_NAME}
    PRIVATE
        ${PROJECT_SOURCES}
)

# Set include directories
target_include_directories(${PROJECT_NAME}
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        ${eigen_SOURCE_DIR}
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        raylib
)

# Platform-specific settings
if(APPLE)
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            "-framework IOKit"
            "-framework Cocoa"
            "-framework OpenGL"
    )
elseif(UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            m
            pthread
            dl
            GL
            rt
            X11
    )
endif()

# Install configuration
include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Enable testing
enable_testing()

# Print configuration summary
message(STATUS "")
message(STATUS "Configuration summary")
message(STATUS "--------------------")
message(STATUS "Project version: ${CMAKE_PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
