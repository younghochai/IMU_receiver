
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(C_vtkMoCap)

find_package(VTK COMPONENTS 
  CommonColor
  CommonCore
  FiltersSources
  InteractionStyle
  RenderingContextOpenGL2
  RenderingCore
  RenderingFreeType
  RenderingGL2PSOpenGL2
  RenderingOpenGL2
  FiltersTexture
  ImagingCore
  ImagingSources     
)

if (NOT VTK_FOUND)
  message(FATAL_ERROR "C_vtkMoCap: Unable to find the VTK build folder.")
endif()

# Prevent a "command line is too long" failure in Windows.
set(CMAKE_NINJA_FORCE_RESPONSE_FILE "ON" CACHE BOOL "Force Ninja to use response files.")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(C_vtkMoCap MACOSX_BUNDLE vtk_main.cxx)
  target_link_libraries(C_vtkMoCap PRIVATE ${VTK_LIBRARIES}
)
# vtk_module_autoinit is needed
vtk_module_autoinit(
  TARGETS C_vtkMoCap
  MODULES ${VTK_LIBRARIES}
)
