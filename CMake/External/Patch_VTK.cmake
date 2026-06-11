if(NOT DEFINED VTK_SOURCE_DIR)
  message(FATAL_ERROR "VTK_SOURCE_DIR is required")
endif()

set(zutil_header "${VTK_SOURCE_DIR}/ThirdParty/zlib/vtkzlib/zutil.h")
file(READ "${zutil_header}" zutil_content)

string(REPLACE
"#if defined(MACOS) || defined(TARGET_OS_MAC)"
"#if (defined(MACOS) || defined(TARGET_OS_MAC)) && !defined(__APPLE__)"
zutil_content "${zutil_content}")

file(WRITE "${zutil_header}" "${zutil_content}")

set(pngpriv_header "${VTK_SOURCE_DIR}/ThirdParty/png/vtkpng/pngpriv.h")
file(READ "${pngpriv_header}" pngpriv_content)

string(REPLACE
"defined(THINK_C) || defined(__SC__) || defined(TARGET_OS_MAC)"
"defined(THINK_C) || defined(__SC__) || (defined(TARGET_OS_MAC) && !defined(__APPLE__))"
pngpriv_content "${pngpriv_content}")

file(WRITE "${pngpriv_header}" "${pngpriv_content}")

set(octree_node_txx "${VTK_SOURCE_DIR}/Utilities/octree/octree/octree_node.txx")
file(READ "${octree_node_txx}" octree_node_content)

string(REPLACE
"_M_chilren"
"_M_children"
octree_node_content "${octree_node_content}")

file(WRITE "${octree_node_txx}" "${octree_node_content}")
