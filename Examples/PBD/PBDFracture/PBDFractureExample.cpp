/*
** This file is part of the Interactive Medical Simulation Toolkit (iMSTK)
** iMSTK is distributed under the Apache License, Version 2.0.
** See accompanying NOTICE for details.
*/

#include "imstkCamera.h"
#include "imstkDirectionalLight.h"
#include "imstkGeometryUtilities.h"
#include "imstkKeyboardDeviceClient.h"
#include "imstkKeyboardSceneControl.h"
#include "imstkLineMesh.h"
#include "imstkMouseSceneControl.h"
#include "imstkNew.h"
#include "imstkPbdConstraintContainer.h"
#include "imstkPbdModel.h"
#include "imstkPbdModelConfig.h"
#include "imstkPbdObject.h"
#include "imstkRenderMaterial.h"
#include "imstkScene.h"
#include "imstkSceneObject.h"
#include "imstkSceneManager.h"
#include "imstkSimulationManager.h"
#include "imstkSimulationUtils.h"
#include "imstkSurfaceMesh.h"
#include "imstkTextVisualModel.h"
#include "imstkTetrahedralMesh.h"
#include "imstkVisualModel.h"
#include "imstkVTKViewer.h"

#include "PBDFractureTear.h"

#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>

using namespace imstk;

using Edge = std::array<int, 2>;
using Face = std::array<int, 3>;

struct PullState
{
    bool   active = false;
    double displacement = 0.0;
    double speed = 0.08;
    double maxDisplacement = 2.4;
};

struct EdgePathInfo
{
    double distanceAlongPath = 0.0;
};

static std::vector<EdgePathInfo>
precomputeEdgePathDistances(const std::vector<Edge>& tetEdges,
                            const std::shared_ptr<TetrahedralMesh> tetMesh,
                            const TearCutSurface& cutSurface)
{
    std::vector<EdgePathInfo> info(tetEdges.size());
    for (size_t i = 0; i < tetEdges.size(); i++)
    {
        const Edge& edge = tetEdges[i];
        const Vec3d p0 = tetMesh->getVertexPosition(edge[0]);
        const Vec3d p1 = tetMesh->getVertexPosition(edge[1]);
        const Vec3d pMid = (p0 + p1) * 0.5;
        const TearCutSurfaceSample sample = cutSurface.closestSample(pMid);
        info[i].distanceAlongPath = sample.distanceAlongPath;
    }
    return info;
}

static int
getGridVertexId(const Vec3i& dim, const int x, const int y, const int z)
{
    return x + dim[0] * (y + dim[1] * z);
}

static Edge
makeEdge(const int i0, const int i1)
{
    return { std::min(i0, i1), std::max(i0, i1) };
}

static std::vector<Edge>
getUniqueTetEdges(const std::shared_ptr<TetrahedralMesh> tetMesh)
{
    std::set<Edge> edges;
    const VecDataArray<int, 4>& tets = *tetMesh->getCells();
    for (int i = 0; i < tets.size(); i++)
    {
        const Vec4i& tet = tets[i];
        edges.insert(makeEdge(tet[0], tet[1]));
        edges.insert(makeEdge(tet[0], tet[2]));
        edges.insert(makeEdge(tet[0], tet[3]));
        edges.insert(makeEdge(tet[1], tet[2]));
        edges.insert(makeEdge(tet[1], tet[3]));
        edges.insert(makeEdge(tet[2], tet[3]));
    }

    return std::vector<Edge>(edges.begin(), edges.end());
}

static bool
hasActiveConstraintForEdge(const std::shared_ptr<PbdConstraintContainer> constraints,
                           const int bodyId,
                           const Edge& edge)
{
    const std::vector<std::shared_ptr<PbdConstraint>> edgeConstraints =
        constraints->getConstraintsForEdge(bodyId, edge[0], edge[1]);

    for (const std::shared_ptr<PbdConstraint>& constraint : edgeConstraints)
    {
        if (constraint->isActive())
        {
            return true;
        }
    }

    return false;
}

static std::shared_ptr<VecDataArray<int, 2>>
buildEdgeCells(const std::vector<Edge>& edges)
{
    auto indices = std::make_shared<VecDataArray<int, 2>>();
    for (const Edge& edge : edges)
    {
        indices->push_back(Vec2i(edge[0], edge[1]));
    }
    return indices;
}

static std::shared_ptr<VecDataArray<int, 2>>
buildActiveEdgeCells(const std::shared_ptr<PbdConstraintContainer> constraints,
                     const int bodyId,
                     const std::vector<Edge>& edges)
{
    auto indices = std::make_shared<VecDataArray<int, 2>>();
    for (const Edge& edge : edges)
    {
        if (hasActiveConstraintForEdge(constraints, bodyId, edge))
        {
            indices->push_back(Vec2i(edge[0], edge[1]));
        }
    }
    return indices;
}

static Face
makeFaceKey(const int i0, const int i1, const int i2)
{
    Face face = { i0, i1, i2 };
    std::sort(face.begin(), face.end());
    return face;
}

static std::shared_ptr<VecDataArray<int, 3>>
buildBoundarySurfaceCells(const std::shared_ptr<TetrahedralMesh> tetMesh)
{
    struct FaceEntry
    {
        Vec3i face = Vec3i::Zero();
        int   count = 0;
    };

    std::map<Face, FaceEntry> faces;
    const VecDataArray<int, 4>& tets = *tetMesh->getCells();
    const std::array<Vec3i, 4> facePattern = {
        Vec3i(0, 1, 2), Vec3i(0, 1, 3), Vec3i(0, 2, 3), Vec3i(1, 2, 3)
    };

    for (int i = 0; i < tets.size(); i++)
    {
        const Vec4i& tet = tets[i];
        for (const Vec3i& pattern : facePattern)
        {
            const Vec3i face(tet[pattern[0]], tet[pattern[1]], tet[pattern[2]]);
            FaceEntry& entry = faces[makeFaceKey(face[0], face[1], face[2])];
            entry.face = face;
            entry.count++;
        }
    }

    auto indices = std::make_shared<VecDataArray<int, 3>>();
    for (const auto& faceEntry : faces)
    {
        if (faceEntry.second.count == 1)
        {
            indices->push_back(faceEntry.second.face);
        }
    }
    return indices;
}

static std::vector<Vec3i>
copySurfaceCells(const std::shared_ptr<SurfaceMesh> surfaceMesh)
{
    std::vector<Vec3i> faces;
    if (surfaceMesh == nullptr || surfaceMesh->getCells() == nullptr)
    {
        return faces;
    }

    const VecDataArray<int, 3>& cells = *surfaceMesh->getCells();
    faces.reserve(cells.size());
    for (int i = 0; i < cells.size(); i++)
    {
        faces.push_back(cells[i]);
    }
    return faces;
}

static std::shared_ptr<SurfaceMesh>
makeBoundarySurfaceMesh(const std::shared_ptr<TetrahedralMesh> tetMesh)
{
    auto surfaceMesh = std::make_shared<SurfaceMesh>();
    auto vertices = std::make_shared<VecDataArray<double, 3>>(*tetMesh->getVertexPositions());
    surfaceMesh->initialize(vertices, buildBoundarySurfaceCells(tetMesh));
    return surfaceMesh;
}

static std::vector<Edge>
getBoundaryEdges(const std::shared_ptr<SurfaceMesh> surfaceMesh)
{
    std::set<Edge> edges;
    const VecDataArray<int, 3>& tris = *surfaceMesh->getCells();
    for (int i = 0; i < tris.size(); i++)
    {
        const Vec3i& tri = tris[i];
        edges.insert(makeEdge(tri[0], tri[1]));
        edges.insert(makeEdge(tri[0], tri[2]));
        edges.insert(makeEdge(tri[1], tri[2]));
    }
    return std::vector<Edge>(edges.begin(), edges.end());
}

static bool
isBoundaryFaceInActiveTearRegion(const Vec3i& face,
                                 const std::vector<Vec3d>& restPositions,
                                 const TearCutSurface& cutSurface,
                                 const TearState& tearState)
{
    if (tearState.frontDistance <= 0.0)
    {
        return false;
    }

    const Vec3d centroid =
        (restPositions[face[0]] + restPositions[face[1]] + restPositions[face[2]]) / 3.0;
    std::array<Vec3d, 4> samplePoints = {
        restPositions[face[0]],
        restPositions[face[1]],
        restPositions[face[2]],
        centroid
    };
    for (const Vec3d& point : samplePoints)
    {
        const TearCutSurfaceSample sample = cutSurface.closestSample(point);
        if (sample.distanceAlongPath <= tearState.frontDistance + tearState.tearRadius
            && sample.distanceToSurface <= tearState.tearRadius * 2.0)
        {
            return true;
        }
    }
    return false;
}

struct ClippedBoundaryVertex
{
    Vec3d restPosition = Vec3d::Zero();
    Vec3d position = Vec3d::Zero();
    double side = 0.0;
};

static ClippedBoundaryVertex
interpolateBoundaryVertex(const ClippedBoundaryVertex& a,
                          const ClippedBoundaryVertex& b,
                          const TearCutSurface& cutSurface,
                          const double sideSign)
{
    const double denominator = std::abs(a.side) + std::abs(b.side);
    const double t = denominator > 1e-12 ? std::abs(a.side) / denominator : 0.5;
    const Vec3d restCutPosition = (1.0 - t) * a.restPosition + t * b.restPosition;
    const ClippedBoundaryVertex& sideAnchor = (a.side * sideSign >= 0.0) ? a : b;
    const TearCutSurfaceSample sample = cutSurface.closestSample(restCutPosition);
    const Vec3d restOffset = restCutPosition - sideAnchor.restPosition;
    const Vec3d tangentialOffset = restOffset - sample.normal * restOffset.dot(sample.normal);

    ClippedBoundaryVertex result;
    result.restPosition = restCutPosition;
    result.position     = sideAnchor.position + tangentialOffset;
    result.side         = 0.0;
    return result;
}

static std::vector<ClippedBoundaryVertex>
clipBoundaryFaceToSide(const std::array<ClippedBoundaryVertex, 3>& inputVertices,
                       const TearCutSurface& cutSurface,
                       const double sideSign)
{
    std::vector<ClippedBoundaryVertex> output;
    output.reserve(4);

    for (int i = 0; i < 3; i++)
    {
        const ClippedBoundaryVertex& current = inputVertices[i];
        const ClippedBoundaryVertex& next = inputVertices[(i + 1) % 3];
        const bool currentInside = current.side * sideSign >= 0.0;
        const bool nextInside = next.side * sideSign >= 0.0;

        if (currentInside && nextInside)
        {
            output.push_back(next);
        }
        else if (currentInside && !nextInside)
        {
            output.push_back(interpolateBoundaryVertex(current, next, cutSurface, sideSign));
        }
        else if (!currentInside && nextInside)
        {
            output.push_back(interpolateBoundaryVertex(current, next, cutSurface, sideSign));
            output.push_back(next);
        }
    }

    return output;
}

static void
appendBoundaryPolygon(const std::vector<ClippedBoundaryVertex>& polygon,
                      std::shared_ptr<VecDataArray<double, 3>> vertices,
                      std::shared_ptr<VecDataArray<int, 3>> triangles,
                      const double windingSign)
{
    if (polygon.size() < 3)
    {
        return;
    }

    const int firstVertex = vertices->size();
    for (const ClippedBoundaryVertex& vertex : polygon)
    {
        vertices->push_back(vertex.position);
    }

    for (int i = 1; i + 1 < static_cast<int>(polygon.size()); i++)
    {
        if (windingSign >= 0.0)
        {
            triangles->push_back(Vec3i(firstVertex, firstVertex + i, firstVertex + i + 1));
        }
        else
        {
            triangles->push_back(Vec3i(firstVertex + i + 1, firstVertex + i, firstVertex));
        }
    }
}

static void
rebuildVisibleBoundarySurfaceMesh(const std::shared_ptr<SurfaceMesh> surfaceMesh,
                                  const std::vector<Vec3i>& boundaryFaces,
                                  const std::vector<Vec3d>& restPositions,
                                  const VecDataArray<double, 3>& currentPositions,
                                  const TearCutSurface& cutSurface,
                                  const TearState& tearState)
{
    auto vertices = std::make_shared<VecDataArray<double, 3>>();
    auto triangles = std::make_shared<VecDataArray<int, 3>>();
    vertices->reserve(boundaryFaces.size() * 4);
    triangles->reserve(boundaryFaces.size() * 2);

    for (const Vec3i& face : boundaryFaces)
    {
        std::array<ClippedBoundaryVertex, 3> faceVertices;
        bool hasPositive = false;
        bool hasNegative = false;
        for (int i = 0; i < 3; i++)
        {
            const int vertexId = face[i];
            faceVertices[i].restPosition = restPositions[vertexId];
            faceVertices[i].position = currentPositions[vertexId];
            faceVertices[i].side = cutSurface.signedDistance(restPositions[vertexId]);
            hasPositive = hasPositive || faceVertices[i].side >= 0.0;
            hasNegative = hasNegative || faceVertices[i].side <= 0.0;
        }

        if (!hasPositive || !hasNegative
            || !isBoundaryFaceInActiveTearRegion(face, restPositions, cutSurface, tearState))
        {
            appendBoundaryPolygon(
                std::vector<ClippedBoundaryVertex>(faceVertices.begin(), faceVertices.end()),
                vertices,
                triangles,
                1.0);
            continue;
        }

        appendBoundaryPolygon(
            clipBoundaryFaceToSide(faceVertices, cutSurface, 1.0),
            vertices,
            triangles,
            1.0);
        appendBoundaryPolygon(
            clipBoundaryFaceToSide(faceVertices, cutSurface, -1.0),
            vertices,
            triangles,
            -1.0);
    }

    surfaceMesh->setVertexPositions(vertices);
    surfaceMesh->setCells(triangles);
    vertices->postModified();
    triangles->postModified();
    surfaceMesh->postModified();
}

static void
refreshBoundarySurfaceVisual(const std::shared_ptr<SurfaceMesh> surfaceMesh,
                             const std::vector<Vec3i>& boundaryFaces,
                             const std::vector<Vec3d>& restPositions,
                             const VecDataArray<double, 3>& currentPositions,
                             const TearCutSurface& cutSurface,
                             const TearState& tearState)
{
    rebuildVisibleBoundarySurfaceMesh(
        surfaceMesh,
        boundaryFaces,
        restPositions,
        currentPositions,
        cutSurface,
        tearState);
}

static void
refreshEdgeVisual(const std::shared_ptr<LineMesh> edgeMesh,
                  const std::shared_ptr<PbdConstraintContainer> constraints,
                  const int bodyId,
                  const std::vector<Edge>& boundaryEdges)
{
    edgeMesh->setCells(buildActiveEdgeCells(constraints, bodyId, boundaryEdges));
    edgeMesh->getCells()->postModified();
    edgeMesh->postModified();
}

static TearPath
makePresetTearPath()
{
    return TearPath({
        Vec3d(3.55, 0.12, -0.15),
        Vec3d(2.75, 0.12, -0.15),
        Vec3d(1.45, 0.12, 0.35),
        Vec3d(0.0, 0.12, -0.20),
        Vec3d(-1.35, 0.12, 0.25),
        Vec3d(-2.65, 0.12, -0.75) });
}

static std::shared_ptr<LineMesh>
makeTearPathLineMesh(const TearPath& path)
{
    auto vertices = std::make_shared<VecDataArray<double, 3>>();
    for (const Vec3d& point : path.getPoints())
    {
        vertices->push_back(point);
    }

    auto indices = std::make_shared<VecDataArray<int, 2>>();
    for (int i = 0; i + 1 < static_cast<int>(path.getPoints().size()); i++)
    {
        indices->push_back(Vec2i(i, i + 1));
    }

    auto lineMesh = std::make_shared<LineMesh>();
    lineMesh->initialize(vertices, indices);
    return lineMesh;
}

static bool
isOnLiftedSide(const Vec3d& point,
               const TearCutSurface& cutSurface,
               const double liftedSideSign)
{
    return cutSurface.signedDistance(point) * liftedSideSign >= 0.0;
}

static bool
isEdgeCutByActiveTearSurface(const std::vector<Vec3d>& restPositions,
                             const TearCutSurface& cutSurface,
                             const TearState& tearState,
                             const Edge& edge)
{
    const Vec3d& p0 = restPositions[edge[0]];
    const Vec3d& p1 = restPositions[edge[1]];
    const Vec3d  pMid = (p0 + p1) * 0.5;

    const TearCutSurfaceSample sample = cutSurface.closestSample(pMid);
    if (sample.distanceAlongPath > tearState.frontDistance + tearState.tearRadius
        || sample.distanceToSurface > tearState.tearRadius)
    {
        return false;
    }

    const double side0 = cutSurface.signedDistance(p0);
    const double side1 = cutSurface.signedDistance(p1);
    return side0 * side1 <= 0.0;
}

static size_t
deactivateEdgesNearTearFront(const std::shared_ptr<PbdConstraintContainer> constraints,
                             const int bodyId,
                             const std::vector<Edge>& tetEdges,
                             const std::vector<EdgePathInfo>& edgePathInfo,
                             const std::vector<Vec3d>& restPositions,
                             const TearCutSurface& cutSurface,
                             const TearState& tearState,
                             const double prevFrontDistance)
{
    size_t deactivatedCount = 0;
    const double windowMin = prevFrontDistance - tearState.tearRadius;
    const double windowMax = tearState.frontDistance + tearState.tearRadius;

    for (size_t i = 0; i < tetEdges.size(); i++)
    {
        const double dist = edgePathInfo[i].distanceAlongPath;
        if (dist < windowMin || dist > windowMax)
        {
            continue;
        }

        if (!isEdgeCutByActiveTearSurface(restPositions, cutSurface, tearState, tetEdges[i]))
        {
            continue;
        }

        const Edge& edge = tetEdges[i];
        const size_t edgeDeactivatedCount = constraints->deactivateConstraintsForEdge(bodyId, edge[0], edge[1]);
        if (edgeDeactivatedCount > 0)
        {
            deactivatedCount += edgeDeactivatedCount;
        }
    }

    return deactivatedCount;
}

static void
applyCodeDrivenPull(const std::shared_ptr<PbdBody> body,
                    const std::vector<int>& pullNodeIds,
                    const std::vector<Vec3d>& pullRestPositions,
                    const PullState& pullState)
{
    if (body == nullptr || body->vertices == nullptr || body->velocities == nullptr)
    {
        return;
    }

    const Vec3d pullOffset(0.0, pullState.displacement, 0.0);
    const Vec3d pullVelocity(0.0, pullState.active ? pullState.speed : 0.0, 0.0);
    for (size_t i = 0; i < pullNodeIds.size(); i++)
    {
        const int nodeId = pullNodeIds[i];
        (*body->vertices)[nodeId] = pullRestPositions[i] + pullOffset;
        (*body->velocities)[nodeId] = pullVelocity;
        if (body->prevVertices != nullptr)
        {
            (*body->prevVertices)[nodeId] = (*body->vertices)[nodeId];
        }
    }
}

static void
applyCodeDrivenLiftedSide(const std::shared_ptr<PbdBody> body,
                          const std::vector<int>& liftedSideNodeIds,
                          const std::vector<Vec3d>& restPositions,
                          const TearCutSurface& cutSurface,
                          const TearState& tearState,
                          const PullState& pullState)
{
    if (body == nullptr || body->vertices == nullptr || body->velocities == nullptr || body->invMasses == nullptr)
    {
        return;
    }

    const Vec3d pullOffset(0.0, pullState.displacement, 0.0);
    for (const int nodeId : liftedSideNodeIds)
    {
        const double invMass = (*body->invMasses)[nodeId];
        if (invMass == 0.0)
        {
            continue;
        }

        const TearCutSurfaceSample sample = cutSurface.closestSample(restPositions[nodeId]);
        const double frontBlend = std::max(
            0.0,
            std::min(1.0, (tearState.frontDistance - sample.distanceAlongPath + tearState.tearRadius)
                / tearState.tearRadius));
        if (frontBlend <= 0.0)
        {
            continue;
        }

        const Vec3d targetPosition = restPositions[nodeId] + pullOffset * frontBlend;
        (*body->vertices)[nodeId] = targetPosition;
        (*body->velocities)[nodeId] = Vec3d(0.0, pullState.active ? pullState.speed * frontBlend : 0.0, 0.0);
        if (body->prevVertices != nullptr)
        {
            (*body->prevVertices)[nodeId] = targetPosition;
        }
    }
}

static void
applyCodeDrivenStationarySide(const std::shared_ptr<PbdBody> body,
                              const std::vector<int>& stationarySideNodeIds,
                              const std::vector<Vec3d>& restPositions)
{
    if (body == nullptr || body->vertices == nullptr || body->velocities == nullptr || body->invMasses == nullptr)
    {
        return;
    }

    for (const int nodeId : stationarySideNodeIds)
    {
        const double invMass = (*body->invMasses)[nodeId];
        if (invMass == 0.0)
        {
            continue;
        }

        (*body->vertices)[nodeId] = restPositions[nodeId];
        (*body->velocities)[nodeId] = Vec3d::Zero();
        if (body->prevVertices != nullptr)
        {
            (*body->prevVertices)[nodeId] = restPositions[nodeId];
        }
    }
}

static void
applyCodeDrivenSeparation(const std::shared_ptr<PbdBody> body,
                          const TearCutSurface& cutSurface,
                          const TearState& tearState)
{
    if (body == nullptr || body->velocities == nullptr || body->vertices == nullptr || body->invMasses == nullptr)
    {
        return;
    }

    for (int i = 0; i < body->vertices->size(); i++)
    {
        const double invMass = (*body->invMasses)[i];
        if (invMass == 0.0)
        {
            continue;
        }

        const Vec3d& p = (*body->vertices)[i];
        const TearCutSurfaceSample sample = cutSurface.closestSample(p);
        if (sample.distanceAlongPath > tearState.frontDistance || sample.distanceToSurface > tearState.tearRadius * 3.0)
        {
            continue;
        }

        const Vec3d normal = sample.normal;
        const double side = cutSurface.signedDistance(p);
        const double sign = (side >= 0.0) ? 1.0 : -1.0;
        (*body->velocities)[i] += normal * sign * 0.01;
    }
}

static std::shared_ptr<PbdObject>
makeFractureBlock(const std::shared_ptr<PbdModel> pbdModel,
                  const TearCutSurface& cutSurface,
                  const double liftedSideSign,
                  std::shared_ptr<TetrahedralMesh>& tetMesh,
                  std::shared_ptr<SurfaceMesh>& surfaceMesh,
                  std::shared_ptr<LineMesh>& edgeMesh,
                  std::vector<Edge>& tetEdges,
                  std::vector<Edge>& boundaryEdges,
                  std::vector<int>& pullNodeIds)
{
    const Vec3d size(7.0, 2.0, 3.0);
    const Vec3i dim(24, 7, 11);
    tetMesh = GeometryUtils::toTetGrid(Vec3d::Zero(), size, dim);
    tetEdges = getUniqueTetEdges(tetMesh);

    std::cout << "PBDFracture: mesh has " << tetMesh->getNumVertices()
              << " vertices, " << tetMesh->getNumCells()
              << " tetrahedra, " << tetEdges.size() << " unique edges." << std::endl;

    imstkNew<PbdObject> tissueObj("XPBD fracture block");
    tissueObj->setPhysicsGeometry(tetMesh);
    tissueObj->setDynamicalModel(pbdModel);
    tissueObj->getPbdBody()->uniformMassValue = 0.08;

    std::set<int> fixedNodeIds;
    for (int z = 0; z < dim[2]; z++)
    {
        for (int y = 0; y < dim[1]; y++)
        {
            const int x = 0;
            fixedNodeIds.insert(getGridVertexId(dim, x, y, z));
        }
    }
    for (int z = 0; z < dim[2]; z++)
    {
        for (int y = dim[1] - 2; y < dim[1]; y++)
        {
            for (int x = dim[0] - 2; x < dim[0]; x++)
            {
                const int nodeId = getGridVertexId(dim, x, y, z);
                if (!isOnLiftedSide(tetMesh->getVertexPosition(nodeId), cutSurface, liftedSideSign))
                {
                    continue;
                }
                fixedNodeIds.insert(nodeId);
                pullNodeIds.push_back(nodeId);
            }
        }
    }
    tissueObj->getPbdBody()->fixedNodeIds.assign(fixedNodeIds.begin(), fixedNodeIds.end());
    std::cout << "PBDFracture: pull handle has " << pullNodeIds.size()
              << " fixed vertices on the upper-right edge." << std::endl;

    const int bodyId = tissueObj->getPbdBody()->bodyHandle;
    pbdModel->getConfig()->enableDistanceConstraint(1.0e4, 1.0, bodyId);
    pbdModel->getConfig()->setBodyDamping(bodyId, 0.05, 0.0);

    surfaceMesh = makeBoundarySurfaceMesh(tetMesh);
    boundaryEdges = getBoundaryEdges(surfaceMesh);

    auto tetMaterial = std::make_shared<RenderMaterial>();
    tetMaterial->setDisplayMode(RenderMaterial::DisplayMode::Surface);
    tetMaterial->setColor(Color(0.6, 0.78, 1.0));
    tetMaterial->setOpacity(0.28);
    tetMaterial->setLineWidth(1.0);
    tetMaterial->setBackFaceCulling(false);

    imstkNew<VisualModel> tetVisual;
    tetVisual->setGeometry(surfaceMesh);
    tetVisual->setRenderMaterial(tetMaterial);
    tissueObj->addVisualModel(tetVisual);

    edgeMesh = std::make_shared<LineMesh>();
    edgeMesh->initialize(tetMesh->getVertexPositions(), buildEdgeCells(boundaryEdges));

    auto edgeMaterial = std::make_shared<RenderMaterial>();
    edgeMaterial->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);
    edgeMaterial->setColor(Color::Orange);
    edgeMaterial->setOpacity(0.55);
    edgeMaterial->setLineWidth(1.0);

    imstkNew<VisualModel> edgeVisual;
    edgeVisual->setGeometry(edgeMesh);
    edgeVisual->setRenderMaterial(edgeMaterial);
    tissueObj->addVisualModel(edgeVisual);

    return tissueObj;
}

int
main()
{
    Logger::startLogger();

    imstkNew<Scene> scene("PBDFracture");
    scene->getActiveCamera()->setPosition(0.0, 4.0, 11.0);
    scene->getActiveCamera()->setFocalPoint(0.0, 0.0, 0.0);
    scene->getActiveCamera()->setViewUp(0.0, 1.0, 0.0);

    imstkNew<PbdModel> pbdModel;
    pbdModel->getConfig()->m_doPartitioning = false;
    pbdModel->getConfig()->m_gravity    = Vec3d::Zero();
    pbdModel->getConfig()->m_dt         = 0.001;
    pbdModel->getConfig()->m_iterations = 4;
    pbdModel->getConfig()->m_solverType = PbdConstraint::SolverType::xPBD;

    const TearPath tearPath = makePresetTearPath();
    constexpr double liftedSideSign = 1.0;
    constexpr double kMaxTearFrontRatio = 0.76;
    TearState tearState;
    const TearCutSurface cutSurface(tearPath, tearState.halfThickness, TearCutSurface::Mode::HorizontalLayer);
    const double maxTearFrontDistance = tearPath.getLength() * kMaxTearFrontRatio;

    std::shared_ptr<TetrahedralMesh> tetMesh;
    std::shared_ptr<SurfaceMesh> tissueSurfaceMesh;
    std::shared_ptr<LineMesh> edgeMesh;
    std::vector<Edge> tetEdges;
    std::vector<Edge> boundaryEdges;
    std::vector<int> pullNodeIds;
    std::shared_ptr<PbdObject> tissueObj = makeFractureBlock(
        pbdModel, cutSurface, liftedSideSign,
        tetMesh, tissueSurfaceMesh, edgeMesh, tetEdges, boundaryEdges, pullNodeIds);
    scene->addSceneObject(tissueObj);
    const std::vector<Vec3i> boundaryFaces = copySurfaceCells(tissueSurfaceMesh);

    std::vector<Vec3d> restPositions;
    restPositions.reserve(tetMesh->getNumVertices());
    std::vector<Vec3d> pullRestPositions;
    pullRestPositions.reserve(pullNodeIds.size());
    const VecDataArray<double, 3>& initialVertices = *tetMesh->getVertexPositions();
    for (int i = 0; i < initialVertices.size(); i++)
    {
        restPositions.push_back(initialVertices[i]);
    }
    for (const int nodeId : pullNodeIds)
    {
        pullRestPositions.push_back(initialVertices[nodeId]);
    }

    std::vector<int> liftedSideNodeIds;
    liftedSideNodeIds.reserve(initialVertices.size());
    std::vector<int> stationarySideNodeIds;
    stationarySideNodeIds.reserve(initialVertices.size());
    for (int i = 0; i < initialVertices.size(); i++)
    {
        if (isOnLiftedSide(restPositions[i], cutSurface, liftedSideSign))
        {
            liftedSideNodeIds.push_back(i);
        }
        else
        {
            stationarySideNodeIds.push_back(i);
        }
    }
    std::cout << "PBDFracture: lifted side has " << liftedSideNodeIds.size()
              << " vertices; stationary side has " << stationarySideNodeIds.size()
              << " vertices; pull handle uses " << pullNodeIds.size()
              << " vertices on that side." << std::endl;

    imstkNew<SceneObject> tearPathObj("Preset tear path");
    {
        auto pathMaterial = std::make_shared<RenderMaterial>();
        pathMaterial->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);
        pathMaterial->setColor(Color(0.1, 0.9, 1.0));
        pathMaterial->setLineWidth(5.0);

        imstkNew<VisualModel> pathVisual;
        pathVisual->setGeometry(makeTearPathLineMesh(tearPath));
        pathVisual->setRenderMaterial(pathMaterial);
        tearPathObj->addVisualModel(pathVisual);
    }
    scene->addSceneObject(tearPathObj);

    PbdFractureSurfaceMeshBuilder stationaryFractureSurfaceBuilder;
    PbdFractureSurfaceMeshBuilder liftedFractureSurfaceBuilder;
    PullState pullState;
    const std::vector<EdgePathInfo> edgePathInfo =
        precomputeEdgePathDistances(tetEdges, tetMesh, cutSurface);
    stationaryFractureSurfaceBuilder.updateTetCutSide(
        tetMesh, restPositions, *tetMesh->getVertexPositions(),
        cutSurface, tearState.frontDistance, tearState.tearRadius,
        -liftedSideSign, 0.0);
    liftedFractureSurfaceBuilder.updateTetCutSide(
        tetMesh, restPositions, *tetMesh->getVertexPositions(),
        cutSurface, tearState.frontDistance, tearState.tearRadius,
        liftedSideSign, 0.0);

    imstkNew<SceneObject> stationaryFractureSurfaceObj("Stationary fracture surface visual");
    {
        auto stationaryFractureSurfaceMaterial = std::make_shared<RenderMaterial>();
        stationaryFractureSurfaceMaterial->setDisplayMode(RenderMaterial::DisplayMode::Surface);
        stationaryFractureSurfaceMaterial->setColor(Color(0.9, 0.08, 0.02));
        stationaryFractureSurfaceMaterial->setOpacity(0.82);
        stationaryFractureSurfaceMaterial->setBackFaceCulling(false);

        imstkNew<VisualModel> stationaryFractureSurfaceVisual;
        stationaryFractureSurfaceVisual->setGeometry(stationaryFractureSurfaceBuilder.getSurfaceMesh());
        stationaryFractureSurfaceVisual->setRenderMaterial(stationaryFractureSurfaceMaterial);
        stationaryFractureSurfaceObj->addVisualModel(stationaryFractureSurfaceVisual);
    }
    scene->addSceneObject(stationaryFractureSurfaceObj);

    imstkNew<SceneObject> liftedFractureSurfaceObj("Lifted fracture surface visual");
    {
        auto liftedFractureSurfaceMaterial = std::make_shared<RenderMaterial>();
        liftedFractureSurfaceMaterial->setDisplayMode(RenderMaterial::DisplayMode::Surface);
        liftedFractureSurfaceMaterial->setColor(Color(1.0, 0.42, 0.04));
        liftedFractureSurfaceMaterial->setOpacity(0.86);
        liftedFractureSurfaceMaterial->setBackFaceCulling(false);

        imstkNew<VisualModel> liftedFractureSurfaceVisual;
        liftedFractureSurfaceVisual->setGeometry(liftedFractureSurfaceBuilder.getSurfaceMesh());
        liftedFractureSurfaceVisual->setRenderMaterial(liftedFractureSurfaceMaterial);
        liftedFractureSurfaceObj->addVisualModel(liftedFractureSurfaceVisual);
    }
    scene->addSceneObject(liftedFractureSurfaceObj);

    imstkNew<DirectionalLight> light;
    light->setFocalPoint(Vec3d(-1.0, -1.0, -1.0));
    light->setIntensity(1.0);
    scene->addLight("Light", light);

    imstkNew<VTKViewer> viewer;
    viewer->setActiveScene(scene);
    viewer->setInfoLevel(0);
    viewer->setWindowTitle("PBD Fracture - Code-driven tearing");

    imstkNew<SceneManager> sceneManager;
    sceneManager->setActiveScene(scene);
    sceneManager->pause();

    imstkNew<SimulationManager> driver;
    driver->addModule(viewer);
    driver->addModule(sceneManager);
    driver->setDesiredDt(1.0 / 240.0);

    scene->addSceneObject(SimulationUtils::createDefaultSceneControl(driver));

    imstkNew<SceneObject> diagnosticsObj("PBDFracture diagnostics");
    auto diagnosticsText = diagnosticsObj->addComponent<TextVisualModel>("PBDFractureDiagnosticsText");
    diagnosticsText->setPosition(TextVisualModel::DisplayPosition::LowerLeft);
    diagnosticsText->setFontSize(24.0);
    diagnosticsText->setTextColor(Color::White);
    diagnosticsText->setText("FPS: -- | sim dt: -- ms | pull: -- | tear: --");

    double latestSceneDt = driver->getDesiredDt();
    double visualDtAccum = 0.0;
    int visualFrameCount = 0;
    auto diagnosticsUpdate = diagnosticsObj->addComponent<LambdaBehaviour>("PBDFractureDiagnosticsUpdate");
    diagnosticsUpdate->setVisualUpdate([&, diagnosticsText](const double& visualDt)
        {
            visualDtAccum += visualDt;
            visualFrameCount++;
            if (visualDtAccum < 0.25)
            {
                return;
            }

            const double visualFps = static_cast<double>(visualFrameCount) / visualDtAccum;
            const double tearRatio = (maxTearFrontDistance > 0.0) ?
                100.0 * tearState.frontDistance / maxTearFrontDistance : 100.0;

            std::ostringstream stream;
            stream << std::fixed << std::setprecision(1)
                   << "FPS: " << visualFps
                   << " | sim dt: " << latestSceneDt * 1000.0 << " ms"
                   << " | pull: " << pullState.displacement << " / " << pullState.maxDisplacement
                   << " | tear: " << tearRatio << "%";
            diagnosticsText->setText(stream.str());

            visualDtAccum = 0.0;
            visualFrameCount = 0;
        });
    scene->addSceneObject(diagnosticsObj);

    double                lastVisualUpdateFrontDistance = 0.0;
    double                lastConsoleLogTime = 0.0;
    bool                  visualTopologyRefreshPending = true;
    bool                  fractureSurfaceRefreshPending = true;
    constexpr double      kVisualUpdateMinStep = 0.03;
    constexpr double      kConsoleLogInterval = 0.5;
    int                   simTickCount = 0;

    connect<Event>(sceneManager, &SceneManager::preUpdate, [&](Event*)
        {
            const int bodyId = tissueObj->getPbdBody()->bodyHandle;
            std::shared_ptr<PbdBody> body = pbdModel->getBody(bodyId);
            const double dt = sceneManager->getDt();
            latestSceneDt = dt;
            pbdModel->getConfig()->m_dt = dt;
            const double previousFrontDistance = tearState.frontDistance;

            if (pullState.active)
            {
                pullState.displacement = std::min(
                    pullState.maxDisplacement,
                    pullState.displacement + pullState.speed * dt);

                const double pullRatio = (pullState.maxDisplacement > 0.0) ?
                    pullState.displacement / pullState.maxDisplacement : 1.0;
                tearState.frontDistance = maxTearFrontDistance * std::min(1.0, pullRatio);
            }

            applyCodeDrivenPull(body, pullNodeIds, pullRestPositions, pullState);
            if (tearState.frontDistance > 0.0)
            {
                applyCodeDrivenLiftedSide(
                    body, liftedSideNodeIds, restPositions, cutSurface, tearState, pullState);
            }

            applyCodeDrivenStationarySide(body, stationarySideNodeIds, restPositions);

            const double frontAdvance = tearState.frontDistance - previousFrontDistance;
            if (frontAdvance > 0.0)
            {
                std::shared_ptr<PbdConstraintContainer> constraints = pbdModel->getConstraints();
                const size_t deactivatedCount = deactivateEdgesNearTearFront(
                    constraints, bodyId, tetEdges, edgePathInfo, restPositions,
                    cutSurface, tearState, previousFrontDistance);

                const double simTime = simTickCount * dt;
                const bool shouldLog = (deactivatedCount > 0)
                    && ((simTime - lastConsoleLogTime >= kConsoleLogInterval)
                        || (pullState.displacement >= pullState.maxDisplacement));

                if (shouldLog)
                {
                    lastConsoleLogTime = simTime;
                    std::cout << "PBDFracture: pull " << pullState.displacement
                              << " / " << pullState.maxDisplacement
                              << ", tear front " << tearState.frontDistance
                              << " / " << maxTearFrontDistance
                              << ", deactivated " << deactivatedCount << " constraints" << std::endl;
                }

                if (tearState.frontDistance - lastVisualUpdateFrontDistance >= kVisualUpdateMinStep
                    || tearState.frontDistance >= maxTearFrontDistance)
                {
                    lastVisualUpdateFrontDistance = tearState.frontDistance;
                    visualTopologyRefreshPending = true;
                }

                fractureSurfaceRefreshPending = true;
            }

            simTickCount++;

            if (pullState.displacement >= pullState.maxDisplacement)
            {
                pullState.active = false;
                tearState.active = false;
            }
        });

    connect<Event>(sceneManager, &SceneManager::postUpdate, [&](Event*)
        {
            const int bodyId = tissueObj->getPbdBody()->bodyHandle;
            std::shared_ptr<PbdBody> body = pbdModel->getBody(bodyId);
            applyCodeDrivenPull(body, pullNodeIds, pullRestPositions, pullState);
            if (tearState.frontDistance > 0.0)
            {
                applyCodeDrivenLiftedSide(
                    body, liftedSideNodeIds, restPositions, cutSurface, tearState, pullState);
            }
            applyCodeDrivenStationarySide(body, stationarySideNodeIds, restPositions);

            if (fractureSurfaceRefreshPending)
            {
                visualTopologyRefreshPending = true;
            }

            if (tissueSurfaceMesh != nullptr)
            {
                if (visualTopologyRefreshPending)
                {
                    refreshBoundarySurfaceVisual(
                        tissueSurfaceMesh,
                        boundaryFaces,
                        restPositions,
                        *tetMesh->getVertexPositions(),
                        cutSurface,
                        tearState);
                    refreshEdgeVisual(edgeMesh, pbdModel->getConstraints(), bodyId, boundaryEdges);
                    visualTopologyRefreshPending = false;
                }
                tissueSurfaceMesh->getVertexPositions()->postModified();
                tissueSurfaceMesh->postModified();
            }
            if (fractureSurfaceRefreshPending)
            {
                stationaryFractureSurfaceBuilder.updateTetCutSide(
                    tetMesh,
                    restPositions,
                    *tetMesh->getVertexPositions(),
                    cutSurface,
                    tearState.frontDistance,
                    tearState.tearRadius,
                    -liftedSideSign,
                    0.0);
                liftedFractureSurfaceBuilder.updateTetCutSide(
                    tetMesh,
                    restPositions,
                    *tetMesh->getVertexPositions(),
                    cutSurface,
                    tearState.frontDistance,
                    tearState.tearRadius,
                    liftedSideSign,
                    0.0);
                fractureSurfaceRefreshPending = false;
            }
            edgeMesh->getVertexPositions()->postModified();
            edgeMesh->postModified();
        });

    queueConnect<KeyEvent>(viewer->getKeyboardDevice(), &KeyboardDeviceClient::keyPress, sceneManager,
        [&](KeyEvent* e)
        {
            if (e->m_key != 'g')
            {
                return;
            }

            if (pullState.displacement >= pullState.maxDisplacement)
            {
                std::cout << "PBDFracture: pull-driven tear already complete." << std::endl;
                return;
            }

            pullState.active = !pullState.active;
            tearState.active = pullState.active;
            std::cout << "PBDFracture: pull-driven tearing "
                      << (pullState.active ? "started/resumed." : "paused.") << std::endl;
        });

    std::cout << "PBDFracture: press 'g' to lift the upper-right handle and drive tearing." << std::endl;
    std::cout << "PBDFracture: cyan line shows tear route; orange LineMesh shows active distance edges; red/orange horizontal tet-cut SurfaceMeshes stop before the left end." << std::endl;

    driver->start();

    return 0;
}
