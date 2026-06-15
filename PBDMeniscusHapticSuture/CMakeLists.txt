###########################################################################
#
# This file is part of the Interactive Medical Simulation Toolkit (iMSTK)
# iMSTK is distributed under the Apache License, Version 2.0.
# See accompanying NOTICE for details.
#
###########################################################################

cmake_minimum_required(VERSION 3.16)

project(Example-PBDMeniscusHapticSuture)

if (NOT DEFINED iMSTK_USE_RENDERING_VTK OR iMSTK_USE_RENDERING_VTK)
  if (COMMAND imstk_add_executable)
    imstk_add_executable(${PROJECT_NAME}
      PBDMeniscusHapticSutureExample.cpp)

    target_link_libraries(${PROJECT_NAME}
      SimulationManager
      Controllers
      Devices
      SceneEntities
      GeometryMappers
      ViewerVTK
      MeshIO)
  else()
    find_package(iMSTK REQUIRED CONFIG)

    add_executable(${PROJECT_NAME}
      PBDMeniscusHapticSutureExample.cpp)

    target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
    target_link_libraries(${PROJECT_NAME}
      PRIVATE
        imstk::SimulationManager
        imstk::Controllers
        imstk::Devices
        imstk::SceneEntities
        imstk::GeometryMappers
        imstk::ViewerVTK
        imstk::MeshIO)
  endif()

  target_compile_definitions(${PROJECT_NAME}
    PRIVATE
      IMPORTED_MENISCUS_VTK_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../banyueban_left_withCut_Cut2.vtk"
      FORCEPS_TOOL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/Data/tools/forceps.obj")

  if (iMSTK_USE_HAPTICS)
    target_compile_definitions(${PROJECT_NAME} PRIVATE iMSTK_USE_HAPTICS)
  endif()

  if (MINGW)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wa,-mbig-obj)
  elseif (MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /bigobj)
  endif()

  set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER Examples/PBD)
endif()
