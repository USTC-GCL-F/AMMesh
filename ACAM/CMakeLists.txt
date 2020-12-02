cmake_minimum_required(VERSION 3.5)

project(ACAM_mesh)

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
        D:/LIBRARY/Eigen
)

target_link_libraries(${PROJECT_NAME}_HW3
    PRIVATE 
        ACAM_mesh
)