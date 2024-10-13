cmake_minimum_required(VERSION 3.10)
project(cpp_practice)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Force CMake to use g++ (based on your initial CMake config)
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

message(STATUS "Using C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Compiler version: ${CMAKE_CXX_COMPILER_VERSION}")

# Enable debug symbols by default when using CMake
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Set compiler flags for the Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

find_package(Eigen3 3.3 REQUIRED)
find_package(Protobuf REQUIRED)

# List of proto files
set(PROTO_FILES
    ${CMAKE_SOURCE_DIR}/proto/message.proto
)

# Generate C++ protobuf files and place them in the build directory
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Include directories (including the directory where the generated files will be placed)
include_directories(
    ${Protobuf_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR}        # Includes generated files in the build directory
    ${CMAKE_SOURCE_DIR}/include
)

# Add the executable with the generated protobuf source files
add_executable(tcp_server
    src/main.cpp
    src/TCPServer.cpp
    src/BearingEKF.cpp
    ${PROTO_SRCS}  # Use the generated .pb.cc files
)

# Link libraries
target_link_libraries(tcp_server Eigen3::Eigen ${Protobuf_LIBRARIES})

# ===================== Python Protobuf Generation =====================

# Output directory for the Python protobuf files
set(PYTHON_PROTO_OUTPUT_DIR ${CMAKE_BINARY_DIR}/generated/python)

# Make sure the output directory exists
file(MAKE_DIRECTORY ${PYTHON_PROTO_OUTPUT_DIR})

# Add a custom command to generate Python protobuf files
add_custom_command(
    OUTPUT ${PYTHON_PROTO_OUTPUT_DIR}/message_pb2.py
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --python_out=${PYTHON_PROTO_OUTPUT_DIR} --proto_path=${CMAKE_SOURCE_DIR}/proto ${CMAKE_SOURCE_DIR}/proto/message.proto
    DEPENDS ${CMAKE_SOURCE_DIR}/proto/message.proto
    COMMENT "Generating Python protobuf files"
)

# Add a custom target for generating Python protobuf files
add_custom_target(generate_python_protobuf ALL
    DEPENDS ${PYTHON_PROTO_OUTPUT_DIR}/message_pb2.py
)

# ===================== Symlink to Python Virtual Environment =====================

# Path to the Python virtual environment (customize this)
set(VENV_PATH "${CMAKE_SOURCE_DIR}/venv")

# Get the Python site-packages directory inside the virtual environment
execute_process(
    COMMAND ${VENV_PATH}/bin/python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Create a symlink from the generated Python protobuf file to the virtualenv's site-packages
add_custom_command(
    TARGET generate_python_protobuf POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${PYTHON_PROTO_OUTPUT_DIR}/message_pb2.py ${PYTHON_SITE_PACKAGES}/message_pb2.py
    COMMENT "Creating symlink for Python protobuf file in virtualenv site-packages"
)