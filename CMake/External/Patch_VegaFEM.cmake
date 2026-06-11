if(NOT DEFINED VEGAFEM_SOURCE_DIR)
  message(FATAL_ERROR "VEGAFEM_SOURCE_DIR is required")
endif()

set(obj_mesh_cmake "${VEGAFEM_SOURCE_DIR}/libraries/objMesh/CMakeLists.txt")
file(READ "${obj_mesh_cmake}" obj_mesh_cmake_content)

string(REPLACE
"  objMeshClose.cpp
)"
"  objMeshClose.cpp
  octree.cpp
  objMeshOctree.cpp
)"
obj_mesh_cmake_content "${obj_mesh_cmake_content}")

string(REPLACE
"  objMeshClose.h
)"
"  objMeshClose.h
  octree.h
  objMeshOctree.h
)"
obj_mesh_cmake_content "${obj_mesh_cmake_content}")

string(REPLACE
"    octree.cpp
    objMeshOctree.cpp
    objMeshOffsetVoxels.cpp"
"    objMeshOffsetVoxels.cpp"
obj_mesh_cmake_content "${obj_mesh_cmake_content}")

string(REPLACE
"    octree.h
    objMeshOctree.h
    objMeshOffsetVoxels.h"
"    objMeshOffsetVoxels.h"
obj_mesh_cmake_content "${obj_mesh_cmake_content}")

file(WRITE "${obj_mesh_cmake}" "${obj_mesh_cmake_content}")

set(octree_source "${VEGAFEM_SOURCE_DIR}/libraries/objMesh/octree.cpp")
file(READ "${octree_source}" octree_content)

string(REPLACE
"void Octree<TriangleClass>::render()
{
  for(int i=0; i<8; i++)"
"void Octree<TriangleClass>::render()
{
#ifdef ENABLE_OpenGL
  for(int i=0; i<8; i++)"
octree_content "${octree_content}")

string(REPLACE
"  if(triangles.size() > 0)
    boundingBox.render();
}

template<class TriangleClass>
void Octree<TriangleClass>::render(int level)"
"  if(triangles.size() > 0)
    boundingBox.render();
#endif
}

template<class TriangleClass>
void Octree<TriangleClass>::render(int level)"
octree_content "${octree_content}")

string(REPLACE
"void Octree<TriangleClass>::render(int level)
{
  for(int i=0; i<8; i++)"
"void Octree<TriangleClass>::render(int level)
{
#ifdef ENABLE_OpenGL
  for(int i=0; i<8; i++)"
octree_content "${octree_content}")

string(REPLACE
"  if ((triangles.size() > 0) && (level == depth))
    boundingBox.render();
}

template<class TriangleClass>
void Octree<TriangleClass>::render(int level, int boxIndex)"
"  if ((triangles.size() > 0) && (level == depth))
    boundingBox.render();
#endif
}

template<class TriangleClass>
void Octree<TriangleClass>::render(int level, int boxIndex)"
octree_content "${octree_content}")

string(REPLACE
"void Octree<TriangleClass>::renderHelper(int level, int boxIndex)
{
  unsigned int  j;"
"void Octree<TriangleClass>::renderHelper(int level, int boxIndex)
{
#ifdef ENABLE_OpenGL
  unsigned int  j;"
octree_content "${octree_content}")

string(REPLACE
"    renderCounter++;
  }
}

template<class TriangleClass>
void Octree<TriangleClass>::deallocate()"
"    renderCounter++;
  }
#endif
}

template<class TriangleClass>
void Octree<TriangleClass>::deallocate()"
octree_content "${octree_content}")

file(WRITE "${octree_source}" "${octree_content}")

set(sparse_solver_cmake "${VEGAFEM_SOURCE_DIR}/libraries/sparseSolver/CMakeLists.txt")
file(READ "${sparse_solver_cmake}" sparse_solver_cmake_content)

string(REPLACE
"    SPOOLESSolver.cpp
    invMKSolver.cpp
)"
"    SPOOLESSolver.cpp
    invMKSolver.cpp
    LagrangeMultiplierSolver.cpp
)"
sparse_solver_cmake_content "${sparse_solver_cmake_content}")

string(REPLACE
"    invMKSolver.h
    linearSolver.h"
"    invMKSolver.h
    LagrangeMultiplierSolver.h
    linearSolver.h"
sparse_solver_cmake_content "${sparse_solver_cmake_content}")

string(REPLACE
"      invZTAZSolver.cpp
      LagrangeMultiplierSolver.cpp
      ZTAZMultiplicator.cpp"
"      invZTAZSolver.cpp
      ZTAZMultiplicator.cpp"
sparse_solver_cmake_content "${sparse_solver_cmake_content}")

string(REPLACE
"      invZTAZSolver.h
      LagrangeMultiplierSolver.h
      ZTAZMultiplicator.h"
"      invZTAZSolver.h
      ZTAZMultiplicator.h"
sparse_solver_cmake_content "${sparse_solver_cmake_content}")

string(REPLACE
"    sparseMatrix
    performanceCounter"
"    sparseMatrix
    performanceCounter
    constrainedDOFs"
sparse_solver_cmake_content "${sparse_solver_cmake_content}")

file(WRITE "${sparse_solver_cmake}" "${sparse_solver_cmake_content}")
