include(imstkAddTest)
imstk_add_test( Constraints )

if (TARGET ConstraintsTests)
  target_include_directories(ConstraintsTests PRIVATE
    ${CMAKE_SOURCE_DIR}/Examples/PBD/PBDFracture
    )
endif()
