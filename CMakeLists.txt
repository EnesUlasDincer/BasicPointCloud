#  minimum required cmake version: 3.1.15 support vs2019
cmake_minimum_required(VERSION 3.1.15)
project(OBPointCloud)

add_executable(${PROJECT_NAME}
  src/PointCloud.cpp
  src/funcs.cpp
  )

set(OrbbecSDK_LIBS_DIR  /home/enes/Desktop/ORBBEC/OrbbecSDK/lib/linux_x64)
set(OrbbecSDK_INCLUDE /home/enes/Desktop/ORBBEC/OrbbecSDK/include) # Adjust this path as necessary
set(OrbbecSDK_INCLUDE_HPP /home/enes/Desktop/ORBBEC/OrbbecSDK/include/libobsensor/hpp) # Adjust this path as necessary
set(Eigen_Folder /home/enes/Desktop/PhD/Repos/eigen) # Adjust this path as necessary

# Find and link TBB
find_package(TBB REQUIRED)

# Link the required libraries from Orbbec SDK
target_link_libraries(${PROJECT_NAME}
  TBB::tbb 
  ${OrbbecSDK_LIBS_DIR}/libOrbbecSDK.so
  ${OrbbecSDK_LIBS_DIR}/libdepthengine.so
  ${OrbbecSDK_LIBS_DIR}/libob_usb.so
  ${OrbbecSDK_LIBS_DIR}/liblive555.so
)

target_include_directories(${PROJECT_NAME} PUBLIC
${OrbbecSDK_INCLUDE}
${OrbbecSDK_INCLUDE_HPP}
${Eigen_Folder}
$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )


  