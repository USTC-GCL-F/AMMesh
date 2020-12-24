cmake_minimum_required(VERSION 3.5)

project(ACAM_mesh)

#设置不生成MinSizeRel;RelWithDebInfo
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

#path
SET(EIGEN_PATH "D:/LIBRARY/Eigen")

############################################################
# Create a library
############################################################

#Generate the static library from the library sources
add_library(ACAM_mesh STATIC 
    src/IOManger.cpp
	src/PolyMesh.cpp
	src/PolyMesh_Base.cpp
	src/PolyMeshIterators.cpp
)

#SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) #设置库文件的输出目录

target_include_directories(ACAM_mesh
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
)


############################################################
# Create an executable
############################################################

# Add an executable with the above sources
add_executable(ACAM_example 
    src/main.cpp
)

# link the new ACAM_example target with the ACAM_mesh target
target_link_libraries( ACAM_example
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 1
############################################################

file (GLOB HW1_FILE
    src/hw1/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW1 ${HW1_FILE})
target_include_directories(${PROJECT_NAME}_HW1
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_HW1
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 2
############################################################

file (GLOB HW2_FILE
    src/hw2/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW2 ${HW2_FILE})
target_include_directories(${PROJECT_NAME}_HW2
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_HW2
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 3
############################################################

file (GLOB HW3_FILE
    src/hw3/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW3 ${HW3_FILE})
target_include_directories(${PROJECT_NAME}_HW3
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_HW3
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 4
############################################################

file (GLOB HW4_FILE
    src/hw4/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW4 ${HW4_FILE})
target_include_directories(${PROJECT_NAME}_HW4
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN_PATH}
)

target_link_libraries(${PROJECT_NAME}_HW4
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 5
############################################################

file (GLOB HW5_FILE
    src/hw5/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW5 ${HW5_FILE})
target_include_directories(${PROJECT_NAME}_HW5
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN_PATH}
)

target_link_libraries(${PROJECT_NAME}_HW5
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 6
############################################################

file (GLOB HW6_FILE
    src/hw6/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW6 ${HW6_FILE})
target_include_directories(${PROJECT_NAME}_HW6
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN_PATH}
)

target_link_libraries(${PROJECT_NAME}_HW6
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 7
############################################################

file (GLOB HW7_FILE
    src/hw7/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW7 ${HW7_FILE})
target_include_directories(${PROJECT_NAME}_HW7
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN_PATH}
)

target_link_libraries(${PROJECT_NAME}_HW7
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 8
############################################################

file (GLOB HW8_FILE
    src/hw8/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW8 ${HW8_FILE})
target_include_directories(${PROJECT_NAME}_HW8
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN_PATH}
)

target_link_libraries(${PROJECT_NAME}_HW8
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 9
############################################################

file (GLOB HW9_FILE
    src/hw9/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW9 ${HW9_FILE})
target_include_directories(${PROJECT_NAME}_HW9
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN_PATH}
)

target_link_libraries(${PROJECT_NAME}_HW9
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 10
############################################################

file (GLOB HW10_FILE
    src/hw10/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW10 ${HW10_FILE})
target_include_directories(${PROJECT_NAME}_HW10
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN_PATH}
)

target_link_libraries(${PROJECT_NAME}_HW10
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 11
############################################################

file (GLOB HW11_FILE
    src/hw11/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW11 ${HW11_FILE})
target_include_directories(${PROJECT_NAME}_HW11
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_HW11
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 12
############################################################

file (GLOB HW12_FILE
    src/hw12/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW12 ${HW12_FILE})
target_include_directories(${PROJECT_NAME}_HW12
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_HW12
    PRIVATE 
        ACAM_mesh
)

############################################################
# Project Homework 13
############################################################

file (GLOB HW13_FILE
    src/hw13/*.cpp
)

# Add an executable with the above sources
add_executable(${PROJECT_NAME}_HW13 ${HW13_FILE})
target_include_directories(${PROJECT_NAME}_HW13
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN_PATH}
)

target_link_libraries(${PROJECT_NAME}_HW13
    PRIVATE 
        ACAM_mesh
)