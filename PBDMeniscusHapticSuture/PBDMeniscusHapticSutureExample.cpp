/*
** This file is part of the Interactive Medical Simulation Toolkit (iMSTK)
** iMSTK is distributed under the Apache License, Version 2.0.
** See accompanying NOTICE for details.
*/

#include "imstkCamera.h"
#include "imstkCapsule.h"
#include "imstkDirectionalLight.h"
#include "imstkGeometryUtilities.h"
#include "imstkIsometricMap.h"
#include "imstkKeyboardDeviceClient.h"
#include "imstkKeyboardSceneControl.h"
#include "imstkLineMesh.h"
#include "imstkMeshIO.h"
#include "imstkNew.h"
#include "imstkPbdContactConstraint.h"
#include "imstkPbdBaryPointToPointConstraint.h"
#include "imstkPbdConstraintContainer.h"
#include "imstkPbdDistanceConstraint.h"
#include "imstkPbdModel.h"
#include "imstkPbdModelConfig.h"
#include "imstkPbdObject.h"
#include "imstkPbdObjectCollision.h"
#include "imstkPbdObjectController.h"
#include "imstkPbdObjectGrasping.h"
#include "imstkPbdSolver.h"
#include "imstkRenderMaterial.h"
#include "imstkScene.h"
#include "imstkSceneManager.h"
#include "imstkSimulationManager.h"
#include "imstkSimulationUtils.h"
#include "imstkSphere.h"
#include "imstkSurfaceMesh.h"
#include "imstkTextVisualModel.h"
#include "imstkTetrahedralMesh.h"
#include "imstkVisualModel.h"
#include "imstkVTKViewer.h"

#ifdef iMSTK_USE_HAPTICS
#include "imstkDeviceManager.h"
#include "imstkDeviceManagerFactory.h"
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <vector>

using namespace imstk;

using Edge = std::array<int, 2>;
using Face = std::array<int, 3>;

static constexpr bool ENABLE_HAPTIC_FORCE_FEEDBACK = false;
static constexpr double HAPTIC_TRANSLATION_SCALING = 35.0;
static constexpr bool ENABLE_NEEDLE_TISSUE_CONTACT = true;
static constexpr bool ENABLE_THREAD_TISSUE_CONTACT = true;
static constexpr bool ENABLE_TOOL_NEEDLE_CONTACT = false;
static constexpr bool ENABLE_TOOL_THREAD_CONTACT = true;
static constexpr bool ENABLE_THREAD_SELF_COLLISION = false;
static constexpr bool ENABLE_MENISCUS_DEFORMATION = true;
static constexpr bool SHOW_MENISCUS_EDGE_OVERLAY = false;
static constexpr double PBD_TIME_STEP = 1.0 / 60.0;
static constexpr double DRIVER_TIME_STEP = 1.0 / 60.0;
static constexpr int PBD_SOLVER_ITERATIONS = 4;
static constexpr double MENISCUS_EDGE_STIFFNESS = 8.0e5;
static constexpr double LAP_TOOL_SCALE = 20.0;
static constexpr double NEEDLE_SCALE = 50.0;
static constexpr double NEEDLE_CURVE_GAIN = 1.35;
static constexpr double GRASP_CAPSULE_RADIUS = 0.05;
static constexpr double GRASP_CAPSULE_LENGTH = 0.18;
static constexpr double MANUAL_NEEDLE_GRASP_TOLERANCE = 0.08;
static constexpr int HAPTIC_BUTTON_COUNT = 4;
static constexpr int HAPTIC_BUTTON_ARM_RELEASE_FRAMES = 10;
static constexpr bool ENABLE_AUTO_SUTURE_ANCHORS = true;
static constexpr int MAX_SUTURE_ANCHORS = 8;
static constexpr double PUNCTURE_SURFACE_DISTANCE = 0.18;
static constexpr double PUNCTURE_SIGN_EPSILON = 0.005;
static constexpr int PUNCTURE_ANCHOR_COOLDOWN_FRAMES = 12;
static constexpr std::array<int, 3> MENISCUS_LEFT_LIP_VERTEX_IDS = { 456, 479, 498 };
static constexpr std::array<int, 3> MENISCUS_RIGHT_LIP_VERTEX_IDS = { 433, 431, 452 };
static constexpr std::array<int, 11> MENISCUS_FRAGMENT_NOTCH_DISPLAY_VERTEX_IDS =
{
    590, 660, 626, 659, 684, 683, 658, 589, 502, 458, 456
};
static constexpr std::array<int, 12> MENISCUS_FRAGMENT_NOTCH_SEED_VERTEX_IDS =
{
    151, 590, 660, 626, 659, 684, 683, 658, 589, 502, 458, 456
};
static constexpr std::array<int, 14> MENISCUS_FRAGMENT_OPPOSITE_NOTCH_VERTEX_IDS =
{
    532, 553, 587, 588, 621, 622, 616, 583, 500, 453, 433, 431, 452, 498
};
static constexpr std::array<int, 14> MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_A =
{
    458, 478, 557, 623, 624, 625, 562, 563, 561, 560, 559, 558, 526, 498
};
static constexpr std::array<int, 13> MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_B =
{
    475, 432, 476, 527, 537, 528, 536, 556, 535, 555, 554, 534, 533
};
static constexpr std::array<int, 5> MENISCUS_FRAGMENT_EXCLUDED_VERTEX_IDS =
{
    58, 96, 97, 56, 76
};
static constexpr double FORCEPS_TEAR_TRIGGER_DISTANCE = 0.32;
static constexpr double FORCEPS_TOOL_LENGTH = 0.36;
static constexpr double MENISCUS_NOTCH_ADJACENCY_BLOCK_RADIUS = 0.30;

#ifndef FORCEPS_TOOL_PATH
#define FORCEPS_TOOL_PATH "D:/Interval/models/jia1.obj"
#endif

enum class DemoStage
{
    ForcepsTear,
    Suture
};

struct FragmentGrabState
{
    bool active = false;
    double selectedSideSign = 1.0;
    Vec3d startToolPosition = Vec3d::Zero();
    std::vector<int> nodeIds;
    std::vector<Vec3d> startNodePositions;
    std::vector<double> weights;
};

struct FracturePathSample
{
    Vec3d point = Vec3d::Zero();
    Vec3d tangent = Vec3d::UnitX();
    Vec3d sideNormal = Vec3d::UnitX();
    double distanceAlongPath = 0.0;
    double distanceToPath = std::numeric_limits<double>::max();
    double signedSideDistance = 0.0;
};

class TearableDistanceConstraint : public PbdDistanceConstraint
{
public:
    void setActive(const bool active)
    {
        m_active.store(active, std::memory_order_relaxed);
        if (!active)
        {
            zeroOutLambda();
        }
    }

    bool isActive() const
    {
        return m_active.load(std::memory_order_relaxed);
    }

    void projectConstraint(PbdState& bodies, const double dt, const SolverType& type) override
    {
        if (!isActive())
        {
            return;
        }
        PbdDistanceConstraint::projectConstraint(bodies, dt, type);
    }

private:
    std::atomic_bool m_active = true;
};

using TearableDistanceConstraintPtr = std::shared_ptr<TearableDistanceConstraint>;

struct FracturePath
{
    std::array<Vec3d, 3> leftLip = {};
    std::array<Vec3d, 3> rightLip = {};
    std::array<Vec3d, 3> center = {};
    std::array<double, 3> cumulativeLength = { 0.0, 0.0, 0.0 };
    double length = 0.0;
    double tearRadius = 0.25;

    FracturePathSample closestSample(const Vec3d& p) const
    {
        FracturePathSample sample;
        double bestDistSq = std::numeric_limits<double>::max();
        for (int i = 0; i < 2; i++)
        {
            const Vec3d a = center[i];
            const Vec3d b = center[i + 1];
            const Vec3d ab = b - a;
            const double denom = ab.squaredNorm();
            const double t = (denom > 1.0e-12) ?
                std::max(0.0, std::min(1.0, (p - a).dot(ab) / denom)) : 0.0;
            const Vec3d q = a + t * ab;
            const double distSq = (p - q).squaredNorm();
            if (distSq >= bestDistSq)
            {
                continue;
            }

            Vec3d sideNormal =
                ((leftLip[i] - rightLip[i]) * (1.0 - t)
                    + (leftLip[i + 1] - rightLip[i + 1]) * t);
            if (sideNormal.squaredNorm() <= 1.0e-12)
            {
                sideNormal = Vec3d::UnitX();
            }
            sideNormal.normalize();

            sample.point = q;
            sample.tangent = (denom > 1.0e-12) ? ab.normalized() : Vec3d::UnitY();
            sample.sideNormal = sideNormal;
            sample.distanceAlongPath = cumulativeLength[i] + t * (cumulativeLength[i + 1] - cumulativeLength[i]);
            sample.distanceToPath = std::sqrt(distSq);
            sample.signedSideDistance = (p - q).dot(sideNormal);
            bestDistSq = distSq;
        }
        return sample;
    }
};

struct MeniscusTissue
{
    std::shared_ptr<PbdObject> object;
    std::shared_ptr<TetrahedralMesh> tetMesh;
    std::shared_ptr<SurfaceMesh> surfaceMesh;
    std::shared_ptr<LineMesh> edgeMesh;
    std::vector<Edge> tetEdges;
    std::vector<Edge> boundaryEdges;
    std::map<Edge, TearableDistanceConstraintPtr> edgeConstraints;
    Vec3d boundsMin = Vec3d::Zero();
    Vec3d boundsMax = Vec3d::Zero();
};

struct FragmentSplit
{
    std::vector<int> fragmentTetIds;
    std::vector<int> mainTetIds;
    std::shared_ptr<TetrahedralMesh> fragmentMesh;
    std::shared_ptr<TetrahedralMesh> mainMesh;
};

static Edge
makeEdge(const int i0, const int i1)
{
    return { std::min(i0, i1), std::max(i0, i1) };
}

static Face
makeFaceKey(const int i0, const int i1, const int i2)
{
    Face face = { i0, i1, i2 };
    std::sort(face.begin(), face.end());
    return face;
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

static void
increaseNeedleCurvature(const std::shared_ptr<VecDataArray<double, 3>> vertices)
{
    if (vertices == nullptr || vertices->size() < 3)
    {
        return;
    }

    int minZIndex = 0;
    int maxZIndex = 0;
    for (int i = 1; i < vertices->size(); i++)
    {
        if ((*vertices)[i].z() < (*vertices)[minZIndex].z())
        {
            minZIndex = i;
        }
        if ((*vertices)[i].z() > (*vertices)[maxZIndex].z())
        {
            maxZIndex = i;
        }
    }

    const double minZ = (*vertices)[minZIndex].z();
    const double maxZ = (*vertices)[maxZIndex].z();
    const double zSpan = maxZ - minZ;
    if (zSpan < 1.0e-8)
    {
        return;
    }

    const double minY = (*vertices)[minZIndex].y();
    const double maxY = (*vertices)[maxZIndex].y();
    for (int i = 0; i < vertices->size(); i++)
    {
        Vec3d& p = (*vertices)[i];
        const double t = (p.z() - minZ) / zSpan;
        const double chordY = minY + t * (maxY - minY);
        p.y() = chordY + (p.y() - chordY) * NEEDLE_CURVE_GAIN;
    }
    vertices->postModified();
}

static std::shared_ptr<VecDataArray<int, 3>>
buildBoundarySurfaceCells(const std::shared_ptr<TetrahedralMesh> tetMesh)
{
    struct FaceEntry
    {
        Vec3i face = Vec3i::Zero();
        int count = 0;
    };

    std::map<Face, FaceEntry> faces;
    const VecDataArray<double, 3>& vertices = *tetMesh->getVertexPositions();
    const VecDataArray<int, 4>& tets = *tetMesh->getCells();
    const std::array<Vec4i, 4> facePattern = {
        Vec4i(0, 1, 2, 3),
        Vec4i(0, 3, 1, 2),
        Vec4i(0, 2, 3, 1),
        Vec4i(1, 3, 2, 0)
    };

    for (int i = 0; i < tets.size(); i++)
    {
        const Vec4i& tet = tets[i];
        for (const Vec4i& pattern : facePattern)
        {
            const Vec3i face(tet[pattern[0]], tet[pattern[1]], tet[pattern[2]]);
            const int oppositeVertex = tet[pattern[3]];
            const Vec3d& p0 = vertices[face[0]];
            const Vec3d& p1 = vertices[face[1]];
            const Vec3d& p2 = vertices[face[2]];
            const Vec3d& p3 = vertices[oppositeVertex];
            const Vec3d normal = (p1 - p0).cross(p2 - p0);
            const Vec3d outward = ((p0 + p1 + p2) / 3.0) - p3;
            const Vec3i outwardFace =
                (normal.dot(outward) >= 0.0) ? face : Vec3i(face[0], face[2], face[1]);
            FaceEntry& entry = faces[makeFaceKey(face[0], face[1], face[2])];
            entry.face = outwardFace;
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

static std::shared_ptr<SurfaceMesh>
makeBoundarySurfaceMesh(const std::shared_ptr<TetrahedralMesh> tetMesh)
{
    auto surfaceMesh = std::make_shared<SurfaceMesh>();
    surfaceMesh->initialize(tetMesh->getVertexPositions(), buildBoundarySurfaceCells(tetMesh));
    surfaceMesh->computeVertexNormals();
    return surfaceMesh;
}

static std::shared_ptr<SurfaceMesh>
makeOriginalBoundarySurfaceMeshForTetSubset(const std::shared_ptr<TetrahedralMesh> sourceMesh,
                                            const std::shared_ptr<TetrahedralMesh> subsetMesh,
                                            const std::vector<int>& selectedTetIds)
{
    struct FaceEntry
    {
        Vec3i face = Vec3i::Zero();
        int count = 0;
    };

    if (sourceMesh == nullptr || subsetMesh == nullptr || selectedTetIds.empty())
    {
        return nullptr;
    }

    const VecDataArray<double, 3>& sourceVertices = *sourceMesh->getVertexPositions();
    const VecDataArray<int, 4>& sourceTets = *sourceMesh->getCells();
    const std::array<Vec4i, 4> facePattern = {
        Vec4i(0, 1, 2, 3),
        Vec4i(0, 3, 1, 2),
        Vec4i(0, 2, 3, 1),
        Vec4i(1, 3, 2, 0)
    };

    std::map<Face, FaceEntry> sourceFaces;
    for (int tetId = 0; tetId < sourceTets.size(); tetId++)
    {
        const Vec4i& tet = sourceTets[tetId];
        for (const Vec4i& pattern : facePattern)
        {
            const Vec3i face(tet[pattern[0]], tet[pattern[1]], tet[pattern[2]]);
            const int oppositeVertex = tet[pattern[3]];
            const Vec3d& p0 = sourceVertices[face[0]];
            const Vec3d& p1 = sourceVertices[face[1]];
            const Vec3d& p2 = sourceVertices[face[2]];
            const Vec3d& p3 = sourceVertices[oppositeVertex];
            const Vec3d normal = (p1 - p0).cross(p2 - p0);
            const Vec3d outward = ((p0 + p1 + p2) / 3.0) - p3;
            const Vec3i outwardFace =
                (normal.dot(outward) >= 0.0) ? face : Vec3i(face[0], face[2], face[1]);
            FaceEntry& entry = sourceFaces[makeFaceKey(face[0], face[1], face[2])];
            entry.face = outwardFace;
            entry.count++;
        }
    }

    std::map<int, int> sourceToSubsetVertex;
    for (const int tetId : selectedTetIds)
    {
        if (tetId < 0 || tetId >= sourceTets.size())
        {
            continue;
        }
        const Vec4i& tet = sourceTets[tetId];
        for (int i = 0; i < 4; i++)
        {
            if (sourceToSubsetVertex.count(tet[i]) == 0)
            {
                sourceToSubsetVertex[tet[i]] = static_cast<int>(sourceToSubsetVertex.size());
            }
        }
    }

    auto surfaceCells = std::make_shared<VecDataArray<int, 3>>();
    for (const auto& faceEntry : sourceFaces)
    {
        if (faceEntry.second.count != 1)
        {
            continue;
        }

        const Vec3i& sourceFace = faceEntry.second.face;
        const auto i0 = sourceToSubsetVertex.find(sourceFace[0]);
        const auto i1 = sourceToSubsetVertex.find(sourceFace[1]);
        const auto i2 = sourceToSubsetVertex.find(sourceFace[2]);
        if (i0 == sourceToSubsetVertex.end()
            || i1 == sourceToSubsetVertex.end()
            || i2 == sourceToSubsetVertex.end())
        {
            continue;
        }
        surfaceCells->push_back(Vec3i(i0->second, i1->second, i2->second));
    }

    auto surfaceMesh = std::make_shared<SurfaceMesh>();
    surfaceMesh->initialize(subsetMesh->getVertexPositions(), surfaceCells);
    surfaceMesh->computeVertexNormals();
    return surfaceMesh;
}

static Vec3d
closestPointOnTriangle(const Vec3d& p, const Vec3d& a, const Vec3d& b, const Vec3d& c)
{
    const Vec3d ab = b - a;
    const Vec3d ac = c - a;
    const Vec3d ap = p - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0)
    {
        return a;
    }

    const Vec3d bp = p - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3)
    {
        return b;
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        return a + (d1 / (d1 - d3)) * ab;
    }

    const Vec3d cp = p - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6)
    {
        return c;
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        return a + (d2 / (d2 - d6)) * ac;
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
        return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
    }

    const double denom = 1.0 / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

static Vec3d
computeTriangleBarycentric(const Vec3d& p, const Vec3d& a, const Vec3d& b, const Vec3d& c)
{
    const Vec3d v0 = b - a;
    const Vec3d v1 = c - a;
    const Vec3d v2 = p - a;
    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);
    const double denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) < 1.0e-12)
    {
        return Vec3d(1.0, 0.0, 0.0);
    }
    const double v = (d11 * d20 - d01 * d21) / denom;
    const double w = (d00 * d21 - d01 * d20) / denom;
    return Vec3d(1.0 - v - w, v, w);
}

struct SurfaceHit
{
    bool valid = false;
    int triangleId = -1;
    Vec3i triangle = Vec3i::Zero();
    Vec3d point = Vec3d::Zero();
    Vec3d barycentric = Vec3d(1.0, 0.0, 0.0);
    double distance = std::numeric_limits<double>::max();
    double signedDistance = 0.0;
};

static SurfaceHit
findClosestSurfaceHit(const std::shared_ptr<SurfaceMesh> surfaceMesh, const Vec3d& p)
{
    SurfaceHit hit;
    if (surfaceMesh == nullptr)
    {
        return hit;
    }

    const VecDataArray<double, 3>& vertices = *surfaceMesh->getVertexPositions();
    const VecDataArray<int, 3>& triangles = *surfaceMesh->getCells();
    for (int i = 0; i < triangles.size(); i++)
    {
        const Vec3i& tri = triangles[i];
        const Vec3d& a = vertices[tri[0]];
        const Vec3d& b = vertices[tri[1]];
        const Vec3d& c = vertices[tri[2]];
        const Vec3d closestPt = closestPointOnTriangle(p, a, b, c);
        const double distance = (p - closestPt).norm();
        if (distance >= hit.distance)
        {
            continue;
        }

        Vec3d normal = (b - a).cross(c - a);
        if (normal.squaredNorm() > 1.0e-12)
        {
            normal.normalize();
        }
        else
        {
            normal = Vec3d::UnitY();
        }

        hit.valid = true;
        hit.triangleId = i;
        hit.triangle = tri;
        hit.point = closestPt;
        hit.barycentric = computeTriangleBarycentric(closestPt, a, b, c);
        hit.distance = distance;
        hit.signedDistance = (p - closestPt).dot(normal);
    }
    return hit;
}

static std::shared_ptr<LineMesh>
makeBoundaryEdgeMesh(const std::shared_ptr<TetrahedralMesh> tetMesh,
                     const std::shared_ptr<SurfaceMesh> surfaceMesh)
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

    auto edgeMesh = std::make_shared<LineMesh>();
    edgeMesh->initialize(
        tetMesh->getVertexPositions(),
        buildEdgeCells(std::vector<Edge>(edges.begin(), edges.end())));
    return edgeMesh;
}

static std::vector<Edge>
getBoundaryEdges(const std::shared_ptr<SurfaceMesh> surfaceMesh)
{
    std::set<Edge> edges;
    if (surfaceMesh == nullptr)
    {
        return {};
    }

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

static void
hideSceneObjectVisuals(const std::shared_ptr<SceneObject> obj)
{
    if (obj == nullptr)
    {
        return;
    }
    for (const auto& visualModel : obj->getComponents<VisualModel>())
    {
        visualModel->hide();
    }
}

static void
showSceneObjectVisuals(const std::shared_ptr<SceneObject> obj)
{
    if (obj == nullptr)
    {
        return;
    }
    for (const auto& visualModel : obj->getComponents<VisualModel>())
    {
        visualModel->show();
    }
}

static FracturePath
makeFracturePathFromTissue(const std::shared_ptr<TetrahedralMesh> tetMesh)
{
    FracturePath path;
    if (tetMesh == nullptr)
    {
        return path;
    }

    const int vertexCount = tetMesh->getNumVertices();
    double maxLipDistance = 0.0;
    for (int i = 0; i < 3; i++)
    {
        const int leftId = MENISCUS_LEFT_LIP_VERTEX_IDS[i];
        const int rightId = MENISCUS_RIGHT_LIP_VERTEX_IDS[i];
        if (leftId < 0 || leftId >= vertexCount || rightId < 0 || rightId >= vertexCount)
        {
            std::cout << "PBDMeniscusHapticSuture: fracture path id out of range: "
                      << leftId << " / " << rightId
                      << " for mesh with " << vertexCount << " vertices." << std::endl;
            return path;
        }

        path.leftLip[i] = tetMesh->getVertexPosition(leftId);
        path.rightLip[i] = tetMesh->getVertexPosition(rightId);
        path.center[i] = (path.leftLip[i] + path.rightLip[i]) * 0.5;
        maxLipDistance = std::max(maxLipDistance, (path.leftLip[i] - path.rightLip[i]).norm());
    }

    path.cumulativeLength[0] = 0.0;
    path.cumulativeLength[1] = (path.center[1] - path.center[0]).norm();
    path.cumulativeLength[2] = path.cumulativeLength[1] + (path.center[2] - path.center[1]).norm();
    path.length = path.cumulativeLength[2];
    path.tearRadius = std::max(0.16, maxLipDistance * 2.5);
    std::cout << "PBDMeniscusHapticSuture: preset fracture line uses left lip 456->479->498, "
              << "right lip 433->431->452, radius " << path.tearRadius << "." << std::endl;
    return path;
}

static bool
isTetOnPositiveFractureSide(const Vec4i& tet,
                            const VecDataArray<double, 3>& vertices,
                            const FracturePath& fracturePath)
{
    Vec3d centroid = Vec3d::Zero();
    for (int i = 0; i < 4; i++)
    {
        centroid += vertices[tet[i]];
    }
    centroid *= 0.25;
    return fracturePath.closestSample(centroid).signedSideDistance >= 0.0;
}

static double
tetSignedVolume(const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& d)
{
    return (b - a).dot((c - a).cross(d - a)) / 6.0;
}

static std::shared_ptr<TetrahedralMesh>
buildTetSubsetMesh(const std::shared_ptr<TetrahedralMesh> sourceMesh,
                   const std::vector<int>& selectedTetIds)
{
    if (sourceMesh == nullptr || selectedTetIds.empty())
    {
        return nullptr;
    }

    const VecDataArray<double, 3>& sourceVertices = *sourceMesh->getVertexPositions();
    const VecDataArray<int, 4>& sourceTets = *sourceMesh->getCells();

    std::map<int, int> remap;
    auto vertices = std::make_shared<VecDataArray<double, 3>>();
    auto cells = std::make_shared<VecDataArray<int, 4>>();
    for (const int tetId : selectedTetIds)
    {
        if (tetId < 0 || tetId >= sourceTets.size())
        {
            continue;
        }

        const Vec4i& sourceTet = sourceTets[tetId];
        Vec4i newTet = Vec4i::Zero();
        for (int i = 0; i < 4; i++)
        {
            const int sourceVertexId = sourceTet[i];
            auto iter = remap.find(sourceVertexId);
            if (iter == remap.end())
            {
                const int newVertexId = vertices->size();
                remap[sourceVertexId] = newVertexId;
                vertices->push_back(sourceVertices[sourceVertexId]);
                newTet[i] = newVertexId;
            }
            else
            {
                newTet[i] = iter->second;
            }
        }
        cells->push_back(newTet);
    }

    auto mesh = std::make_shared<TetrahedralMesh>();
    mesh->initialize(vertices, cells);
    mesh->postModified();
    return mesh;
}

static std::shared_ptr<TetrahedralMesh>
buildLargestFractureComponentMesh(const MeniscusTissue& tissue,
                                  const FracturePath& fracturePath)
{
    if (tissue.tetMesh == nullptr)
    {
        return nullptr;
    }

    const VecDataArray<double, 3>& vertices = *tissue.tetMesh->getVertexPositions();
    const VecDataArray<int, 4>& tets = *tissue.tetMesh->getCells();
    std::vector<int> positiveTetIds;
    std::vector<int> negativeTetIds;
    double positiveVolume = 0.0;
    double negativeVolume = 0.0;

    for (int tetId = 0; tetId < tets.size(); tetId++)
    {
        const Vec4i& tet = tets[tetId];
        const double volume = std::abs(tetSignedVolume(
            vertices[tet[0]], vertices[tet[1]], vertices[tet[2]], vertices[tet[3]]));
        if (isTetOnPositiveFractureSide(tet, vertices, fracturePath))
        {
            positiveTetIds.push_back(tetId);
            positiveVolume += volume;
        }
        else
        {
            negativeTetIds.push_back(tetId);
            negativeVolume += volume;
        }
    }

    const std::vector<int>& selectedTetIds =
        (positiveTetIds.size() > negativeTetIds.size()
            || (positiveTetIds.size() == negativeTetIds.size() && positiveVolume >= negativeVolume)) ?
        positiveTetIds : negativeTetIds;

    std::cout << "PBDMeniscusHapticSuture: fracture split selected "
              << selectedTetIds.size() << " / " << tets.size()
              << " tetrahedra for the main component." << std::endl;
    return buildTetSubsetMesh(tissue.tetMesh, selectedTetIds);
}

static std::shared_ptr<TetrahedralMesh>
buildFractureSideComponentMesh(const MeniscusTissue& tissue,
                               const FracturePath& fracturePath,
                               const double sideSign)
{
    if (tissue.tetMesh == nullptr)
    {
        return nullptr;
    }

    const VecDataArray<double, 3>& vertices = *tissue.tetMesh->getVertexPositions();
    const VecDataArray<int, 4>& tets = *tissue.tetMesh->getCells();
    std::vector<int> selectedTetIds;
    selectedTetIds.reserve(tets.size());
    double selectedVolume = 0.0;

    for (int tetId = 0; tetId < tets.size(); tetId++)
    {
        const Vec4i& tet = tets[tetId];
        Vec3d centroid = Vec3d::Zero();
        for (int i = 0; i < 4; i++)
        {
            centroid += vertices[tet[i]];
        }
        centroid *= 0.25;

        const double signedSide = fracturePath.closestSample(centroid).signedSideDistance;
        if (signedSide * sideSign < 0.0)
        {
            continue;
        }

        selectedTetIds.push_back(tetId);
        selectedVolume += std::abs(tetSignedVolume(
            vertices[tet[0]], vertices[tet[1]], vertices[tet[2]], vertices[tet[3]]));
    }

    std::cout << "PBDMeniscusHapticSuture: prebuilt fracture side "
              << sideSign << " with " << selectedTetIds.size()
              << " / " << tets.size() << " tetrahedra, volume "
              << selectedVolume << "." << std::endl;
    return buildTetSubsetMesh(tissue.tetMesh, selectedTetIds);
}

static bool
isSharedFaceBlockedByFracture(const Vec3i& face,
                              const VecDataArray<double, 3>& vertices,
                              const FracturePath& fracturePath)
{
    const Vec3d centroid = (vertices[face[0]] + vertices[face[1]] + vertices[face[2]]) / 3.0;
    const FracturePathSample sample = fracturePath.closestSample(centroid);
    if (sample.distanceToPath > fracturePath.tearRadius * 3.0
        || sample.distanceAlongPath > fracturePath.length + fracturePath.tearRadius)
    {
        return false;
    }

    bool hasPositive = false;
    bool hasNegative = false;
    for (int i = 0; i < 3; i++)
    {
        const double side = fracturePath.closestSample(vertices[face[i]]).signedSideDistance;
        hasPositive = hasPositive || side >= -fracturePath.tearRadius * 0.08;
        hasNegative = hasNegative || side <= fracturePath.tearRadius * 0.08;
    }
    return hasPositive && hasNegative;
}

static Vec3d
closestPointOnSegment(const Vec3d& p, const Vec3d& a, const Vec3d& b)
{
    const Vec3d ab = b - a;
    const double denom = ab.squaredNorm();
    if (denom < 1.0e-12)
    {
        return a;
    }
    const double t = std::max(0.0, std::min(1.0, (p - a).dot(ab) / denom));
    return a + t * ab;
}

template<size_t N>
static std::vector<Vec3d>
makePathPointsFromVertexIds(const std::shared_ptr<TetrahedralMesh> tetMesh,
                            const std::array<int, N>& ids)
{
    std::vector<Vec3d> points;
    if (tetMesh == nullptr)
    {
        return points;
    }

    points.reserve(ids.size());
    for (const int id : ids)
    {
        if (id < 0 || id >= tetMesh->getNumVertices())
        {
            continue;
        }
        points.push_back(tetMesh->getVertexPosition(id));
    }
    return points;
}

static double
distanceToPolyline(const Vec3d& point,
                   const std::vector<Vec3d>& pathPoints)
{
    if (pathPoints.size() < 2)
    {
        return std::numeric_limits<double>::max();
    }

    double bestDistance = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(pathPoints.size()) - 1; i++)
    {
        bestDistance = std::min(
            bestDistance,
            (point - closestPointOnSegment(point, pathPoints[i], pathPoints[i + 1])).norm());
    }
    return bestDistance;
}

static double
distanceToFragmentNotchPath(const Vec3d& point,
                            const std::shared_ptr<TetrahedralMesh> tetMesh)
{
    return std::min(
        std::min(
            distanceToPolyline(
                point,
                makePathPointsFromVertexIds(tetMesh, MENISCUS_FRAGMENT_NOTCH_SEED_VERTEX_IDS)),
            distanceToPolyline(
                point,
                makePathPointsFromVertexIds(tetMesh, MENISCUS_FRAGMENT_OPPOSITE_NOTCH_VERTEX_IDS))),
        std::min(
            distanceToPolyline(
                point,
                makePathPointsFromVertexIds(tetMesh, MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_A)),
            distanceToPolyline(
                point,
                makePathPointsFromVertexIds(tetMesh, MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_B))));
}

static std::vector<int>
collectFragmentSeedTetIds(const std::shared_ptr<TetrahedralMesh> tetMesh)
{
    std::set<int> seedTetIds;
    if (tetMesh == nullptr)
    {
        return {};
    }

    const VecDataArray<int, 4>& tets = *tetMesh->getCells();
    std::set<int> seedVertices(
        MENISCUS_FRAGMENT_NOTCH_SEED_VERTEX_IDS.begin(),
        MENISCUS_FRAGMENT_NOTCH_SEED_VERTEX_IDS.end());
    seedVertices.insert(
        MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_A.begin(),
        MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_A.end());
    seedVertices.insert(
        MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_B.begin(),
        MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_B.end());
    std::set<int> excludedVertices(
        MENISCUS_FRAGMENT_EXCLUDED_VERTEX_IDS.begin(),
        MENISCUS_FRAGMENT_EXCLUDED_VERTEX_IDS.end());
    for (int tetId = 0; tetId < tets.size(); tetId++)
    {
        const Vec4i& tet = tets[tetId];
        bool hasExcludedVertex = false;
        for (int i = 0; i < 4; i++)
        {
            hasExcludedVertex = hasExcludedVertex || excludedVertices.count(tet[i]) > 0;
        }
        if (hasExcludedVertex)
        {
            continue;
        }

        for (int i = 0; i < 4; i++)
        {
            if (seedVertices.count(tet[i]) > 0)
            {
                seedTetIds.insert(tetId);
                break;
            }
        }
    }
    return std::vector<int>(seedTetIds.begin(), seedTetIds.end());
}

static FragmentSplit
buildSeededFragmentSplit(const MeniscusTissue& tissue,
                         const FracturePath& fracturePath)
{
    (void)fracturePath;
    FragmentSplit split;
    if (tissue.tetMesh == nullptr)
    {
        return split;
    }

    const VecDataArray<int, 4>& tets = *tissue.tetMesh->getCells();
    std::set<int> fragmentVertices(
        MENISCUS_FRAGMENT_NOTCH_SEED_VERTEX_IDS.begin(),
        MENISCUS_FRAGMENT_NOTCH_SEED_VERTEX_IDS.end());
    fragmentVertices.insert(
        MENISCUS_FRAGMENT_OPPOSITE_NOTCH_VERTEX_IDS.begin(),
        MENISCUS_FRAGMENT_OPPOSITE_NOTCH_VERTEX_IDS.end());
    fragmentVertices.insert(
        MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_A.begin(),
        MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_A.end());
    fragmentVertices.insert(
        MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_B.begin(),
        MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_B.end());

    std::set<int> excludedVertices(
        MENISCUS_FRAGMENT_EXCLUDED_VERTEX_IDS.begin(),
        MENISCUS_FRAGMENT_EXCLUDED_VERTEX_IDS.end());

    std::vector<int> fragmentTetIds;
    std::vector<int> mainTetIds;
    fragmentTetIds.reserve(tets.size());
    mainTetIds.reserve(tets.size());
    size_t excludedTetCount = 0;
    for (int tetId = 0; tetId < tets.size(); tetId++)
    {
        const Vec4i& tet = tets[tetId];
        bool hasExcludedVertex = false;
        bool allVerticesExplicit = true;
        for (int i = 0; i < 4; i++)
        {
            hasExcludedVertex = hasExcludedVertex || excludedVertices.count(tet[i]) > 0;
            allVerticesExplicit = allVerticesExplicit && fragmentVertices.count(tet[i]) > 0;
        }

        if (allVerticesExplicit && !hasExcludedVertex)
        {
            fragmentTetIds.push_back(tetId);
        }
        else
        {
            if (allVerticesExplicit && hasExcludedVertex)
            {
                excludedTetCount++;
            }
            mainTetIds.push_back(tetId);
        }
    }

    split.fragmentTetIds = fragmentTetIds;
    split.mainTetIds = mainTetIds;
    split.fragmentMesh = buildTetSubsetMesh(tissue.tetMesh, split.fragmentTetIds);
    split.mainMesh = buildTetSubsetMesh(tissue.tetMesh, split.mainTetIds);

    std::cout << "PBDMeniscusHapticSuture: notch-seeded tear fragment has "
              << split.fragmentTetIds.size() << " tetrahedra from "
              << fragmentVertices.size()
              << " explicit vertices, excluding "
              << excludedTetCount << " protected-boundary tetrahedra; suture main body keeps "
              << split.mainTetIds.size() << " tetrahedra." << std::endl;
    return split;
}

static std::shared_ptr<VecDataArray<int, 3>>
buildFracturedBoundarySurfaceCells(const std::shared_ptr<TetrahedralMesh> tetMesh,
                                   const FracturePath& fracturePath,
                                   const double frontDistance)
{
    auto cells = buildBoundarySurfaceCells(tetMesh);
    auto fracturedCells = std::make_shared<VecDataArray<int, 3>>();
    if (tetMesh == nullptr || cells == nullptr)
    {
        return fracturedCells;
    }

    const VecDataArray<double, 3>& vertices = *tetMesh->getVertexPositions();
    for (int i = 0; i < cells->size(); i++)
    {
        const Vec3i& tri = (*cells)[i];
        const Vec3d centroid = (vertices[tri[0]] + vertices[tri[1]] + vertices[tri[2]]) / 3.0;
        const FracturePathSample sample = fracturePath.closestSample(centroid);
        const bool inCutWindow =
            sample.distanceAlongPath <= frontDistance + fracturePath.tearRadius
            && sample.distanceToPath <= fracturePath.tearRadius * 2.4;
        if (!inCutWindow)
        {
            fracturedCells->push_back(tri);
        }
    }
    return fracturedCells;
}

static void
applyFractureVisualGap(MeniscusTissue& tissue,
                       const FracturePath& fracturePath,
                       const double frontDistance)
{
    if (tissue.surfaceMesh == nullptr || tissue.tetMesh == nullptr)
    {
        return;
    }
    tissue.surfaceMesh->setTriangleIndices(
        buildFracturedBoundarySurfaceCells(tissue.tetMesh, fracturePath, frontDistance));
    tissue.surfaceMesh->computeVertexNormals();
    tissue.surfaceMesh->postModified();
    std::cout << "PBDMeniscusHapticSuture: fracture visual mesh now has "
              << tissue.surfaceMesh->getNumCells()
              << " boundary triangles after removing tear-window faces." << std::endl;
}

static void
applyOriginalBoundarySurface(MeniscusTissue& tissue,
                             const std::shared_ptr<TetrahedralMesh> sourceMesh,
                             const std::vector<int>& selectedTetIds,
                             const std::string& label)
{
    if (tissue.surfaceMesh == nullptr || tissue.tetMesh == nullptr)
    {
        return;
    }

    const int before = tissue.surfaceMesh->getNumCells();
    const std::shared_ptr<SurfaceMesh> originalBoundarySurface =
        makeOriginalBoundarySurfaceMeshForTetSubset(sourceMesh, tissue.tetMesh, selectedTetIds);
    if (originalBoundarySurface == nullptr)
    {
        return;
    }

    tissue.surfaceMesh = originalBoundarySurface;
    tissue.edgeMesh = makeBoundaryEdgeMesh(tissue.tetMesh, tissue.surfaceMesh);
    tissue.boundaryEdges = getBoundaryEdges(tissue.surfaceMesh);
    if (tissue.object != nullptr)
    {
        tissue.object->setVisualGeometry(tissue.surfaceMesh);
        tissue.object->setCollidingGeometry(tissue.surfaceMesh);
    }
    std::cout << "PBDMeniscusHapticSuture: " << label
              << " visible surface triangles " << before << " -> "
              << tissue.surfaceMesh->getNumCells()
              << " after keeping original boundary faces only." << std::endl;
}

static void
centerAndScaleTetMesh(const std::shared_ptr<TetrahedralMesh> tetMesh)
{
    Vec3d boundsMin = Vec3d::Zero();
    Vec3d boundsMax = Vec3d::Zero();
    tetMesh->computeBoundingBox(boundsMin, boundsMax);
    const Vec3d center = (boundsMin + boundsMax) * 0.5;
    const Vec3d extent = boundsMax - boundsMin;
    const double maxExtent = std::max(extent[0], std::max(extent[1], extent[2]));
    const double scale = (maxExtent > 1.0e-8) ? 6.7 / maxExtent : 1.0;

    VecDataArray<double, 3>& vertices = *tetMesh->getVertexPositions();
    for (int i = 0; i < vertices.size(); i++)
    {
        vertices[i] = (vertices[i] - center) * scale;
    }
    vertices.postModified();
    tetMesh->postModified();
}

static std::vector<int>
selectSupportNodes(const std::shared_ptr<TetrahedralMesh> tetMesh,
                   const Vec3d& boundsMin,
                   const Vec3d& boundsMax)
{
    const Vec3d extent = boundsMax - boundsMin;
    const double xMargin = std::max(0.18, extent[0] * 0.075);
    const double zMargin = std::max(0.16, extent[2] * 0.10);

    std::set<int> fixedNodeIds;
    for (int i = 0; i < tetMesh->getNumVertices(); i++)
    {
        const Vec3d p = tetMesh->getVertexPosition(i);
        const bool hornSupport =
            (p[0] < boundsMin[0] + xMargin) || (p[0] > boundsMax[0] - xMargin);
        const bool posteriorRimSupport = p[2] < boundsMin[2] + zMargin;
        if (hornSupport || posteriorRimSupport)
        {
            fixedNodeIds.insert(i);
        }
    }
    return std::vector<int>(fixedNodeIds.begin(), fixedNodeIds.end());
}

static MeniscusTissue
makeMeniscusObjectFromMesh(const std::shared_ptr<PbdModel> pbdModel,
                           const std::shared_ptr<TetrahedralMesh> tetMesh,
                           const std::string& objectName)
{
    MeniscusTissue tissue;
    tissue.tetMesh = tetMesh;
    if (tissue.tetMesh == nullptr)
    {
        return tissue;
    }

    tissue.tetMesh->computeBoundingBox(tissue.boundsMin, tissue.boundsMax);
    tissue.surfaceMesh = makeBoundarySurfaceMesh(tissue.tetMesh);
    tissue.edgeMesh = makeBoundaryEdgeMesh(tissue.tetMesh, tissue.surfaceMesh);
    tissue.tetEdges = getUniqueTetEdges(tissue.tetMesh);
    tissue.boundaryEdges = getBoundaryEdges(tissue.surfaceMesh);

    auto tissueObj = std::make_shared<PbdObject>(objectName);
    tissue.object = tissueObj;
    tissueObj->setPhysicsGeometry(tissue.tetMesh);
    tissueObj->setVisualGeometry(tissue.surfaceMesh);
    tissueObj->setCollidingGeometry(tissue.surfaceMesh);
    tissueObj->setDynamicalModel(pbdModel);
    tissueObj->getPbdBody()->uniformMassValue = 0.06;
    tissueObj->getPbdBody()->fixedNodeIds =
        selectSupportNodes(tissue.tetMesh, tissue.boundsMin, tissue.boundsMax);

    const int bodyId = tissueObj->getPbdBody()->bodyHandle;
    if (ENABLE_MENISCUS_DEFORMATION)
    {
        for (const Edge& edge : tissue.tetEdges)
        {
            auto constraint = std::make_shared<TearableDistanceConstraint>();
            constraint->initConstraint(
                tissue.tetMesh->getVertexPosition(edge[0]),
                tissue.tetMesh->getVertexPosition(edge[1]),
                PbdParticleId(bodyId, edge[0]),
                PbdParticleId(bodyId, edge[1]),
                MENISCUS_EDGE_STIFFNESS);
            pbdModel->getConstraints()->addConstraint(constraint);
            tissue.edgeConstraints[edge] = constraint;
        }
    }
    else
    {
        tissueObj->getPbdBody()->fixedNodeIds.clear();
        tissueObj->getPbdBody()->fixedNodeIds.reserve(tissue.tetMesh->getNumVertices());
        for (int i = 0; i < tissue.tetMesh->getNumVertices(); i++)
        {
            tissueObj->getPbdBody()->fixedNodeIds.push_back(i);
        }
    }
    pbdModel->getConfig()->setBodyDamping(bodyId, 0.055, 0.0);

    std::shared_ptr<RenderMaterial> material = tissueObj->getVisualModel(0)->getRenderMaterial();
    if (material != nullptr)
    {
        material->setDisplayMode(RenderMaterial::DisplayMode::Surface);
        material->setOpacity(1.0);
        material->setBackFaceCulling(false);
        material->setIsDynamicMesh(ENABLE_MENISCUS_DEFORMATION);
        material->setShadingModel(RenderMaterial::ShadingModel::Phong);
        material->setDiffuseColor(Color(0.76, 0.76, 0.72));
    }

    std::cout << "PBDMeniscusHapticSuture: loaded "
              << tissue.tetMesh->getNumVertices() << " vertices, "
              << tissue.tetMesh->getNumCells() << " tetrahedra, "
              << (ENABLE_MENISCUS_DEFORMATION ? tissue.tetEdges.size() : 0)
              << " distance constraints for " << objectName << "." << std::endl;
    std::cout << "PBDMeniscusHapticSuture: fixed "
              << tissueObj->getPbdBody()->fixedNodeIds.size()
              << " support vertices on horns/posterior rim." << std::endl;
    return tissue;
}

static MeniscusTissue
makeImportedMeniscusObject(const std::shared_ptr<PbdModel> pbdModel)
{
    std::shared_ptr<TetrahedralMesh> tetMesh = MeshIO::read<TetrahedralMesh>(IMPORTED_MENISCUS_VTK_PATH);
    if (tetMesh == nullptr)
    {
        std::cout << "PBDMeniscusHapticSuture: failed to load tetrahedral mesh "
                  << IMPORTED_MENISCUS_VTK_PATH << std::endl;
        return {};
    }

    centerAndScaleTetMesh(tetMesh);
    MeniscusTissue tissue = makeMeniscusObjectFromMesh(
        pbdModel,
        tetMesh,
        "Imported left meniscus PBD tissue");
    std::cout << "PBDMeniscusHapticSuture: source mesh path "
              << IMPORTED_MENISCUS_VTK_PATH << std::endl;
    return tissue;
}

static std::shared_ptr<PbdObject>
makeLapToolObj(const std::string& name, const std::shared_ptr<PbdModel> model, const Vec3d& initPos)
{
    auto lapTool = std::make_shared<PbdObject>(name);

    const double capsuleLength = 0.3 * LAP_TOOL_SCALE;
    auto toolGeom = std::make_shared<Capsule>(
        Vec3d(0.0, 0.0, capsuleLength * 0.5 - 0.005),
        0.002 * LAP_TOOL_SCALE,
        capsuleLength,
        Quatd(Rotd(PI_2, Vec3d::UnitX())));

    const double lapToolHeadLength = GRASP_CAPSULE_LENGTH;
    auto graspCapsule = std::make_shared<Capsule>(
        Vec3d(0.0, 0.0, lapToolHeadLength * 0.5),
        GRASP_CAPSULE_RADIUS,
        lapToolHeadLength,
        Quatd::FromTwoVectors(Vec3d::UnitY(), Vec3d::UnitZ()));

    std::shared_ptr<SurfaceMesh> visualGeom =
        MeshIO::read<SurfaceMesh>(std::string(DEMO_IMSTK_DATA_ROOT)
            + "/Surgical Instruments/LapTool/laptool_all_in_one.obj");

    lapTool->setDynamicalModel(model);
    lapTool->setPhysicsGeometry(toolGeom);
    lapTool->setCollidingGeometry(toolGeom);
    if (visualGeom != nullptr)
    {
        visualGeom->scale(LAP_TOOL_SCALE, Geometry::TransformType::ApplyToData);
        lapTool->setVisualGeometry(visualGeom);
        lapTool->setPhysicsToVisualMap(std::make_shared<IsometricMap>(toolGeom, visualGeom));
    }
    else
    {
        lapTool->setVisualGeometry(toolGeom);
    }

    auto graspVisualModel = std::make_shared<VisualModel>();
    graspVisualModel->setGeometry(graspCapsule);
    graspVisualModel->getRenderMaterial()->setIsDynamicMesh(false);
    graspVisualModel->getRenderMaterial()->setOpacity(0.35);
    graspVisualModel->getRenderMaterial()->setColor(Color::Green);
    graspVisualModel->setIsVisible(true);
    lapTool->addVisualModel(graspVisualModel);

    std::shared_ptr<RenderMaterial> material = lapTool->getVisualModel(0)->getRenderMaterial();
    material->setIsDynamicMesh(false);
    material->setMetalness(1.0);
    material->setRoughness(0.2);
    material->setShadingModel(RenderMaterial::ShadingModel::PBR);

    lapTool->getPbdBody()->setRigid(
        initPos,
        5.0,
        Quatd::Identity(),
        Mat3d::Identity() * 0.08);

    auto controller = lapTool->addComponent<PbdObjectController>();
    controller->setControlledObject(lapTool);
    controller->setLinearKs(10000.0);
    controller->setAngularKs(10.0);
    controller->setForceScaling(0.01);
    controller->setSmoothingKernelSize(15);
    controller->setUseForceSmoothening(true);
    controller->setHapticOffset(Vec3d(0.0, 0.0, capsuleLength));

    auto graspCapsuleMap = std::make_shared<IsometricMap>(toolGeom, graspCapsule);
    auto graspCapsuleUpdate = lapTool->addComponent<LambdaBehaviour>(name + "GraspCapsuleUpdate");
    graspCapsuleUpdate->setUpdate([graspCapsuleMap](const double&)
        {
            graspCapsuleMap->update();
        });

    return lapTool;
}

static std::shared_ptr<PbdObject>
makeForcepsToolObj(const std::shared_ptr<PbdModel> model, const Vec3d& initPos)
{
    auto toolObj = std::make_shared<PbdObject>("hapticForcepsTearTool");

    auto toolGeom = std::make_shared<LineMesh>();
    auto vertices = std::make_shared<VecDataArray<double, 3>>(2);
    (*vertices)[0] = Vec3d(0.0, 0.0, 0.0);
    (*vertices)[1] = Vec3d(0.0, FORCEPS_TOOL_LENGTH, 0.0);
    auto indices = std::make_shared<VecDataArray<int, 2>>(1);
    (*indices)[0] = Vec2i(0, 1);
    toolGeom->initialize(vertices, indices);

    toolObj->setDynamicalModel(model);
    toolObj->setPhysicsGeometry(toolGeom);
    toolObj->setCollidingGeometry(toolGeom);

    std::shared_ptr<SurfaceMesh> forcepsMesh = MeshIO::read<SurfaceMesh>(FORCEPS_TOOL_PATH);
    if (forcepsMesh != nullptr)
    {
        Vec3d boundsMin = Vec3d::Zero();
        Vec3d boundsMax = Vec3d::Zero();
        forcepsMesh->computeBoundingBox(boundsMin, boundsMax);
        const Vec3d extent = boundsMax - boundsMin;
        const double maxExtent = std::max(extent[0], std::max(extent[1], extent[2]));
        const double scale = (maxExtent > 1.0e-8) ? FORCEPS_TOOL_LENGTH / maxExtent : 1.0;

        const Vec3d axisBase(
            (boundsMin[0] + boundsMax[0]) * 0.5,
            boundsMin[1],
            (boundsMin[2] + boundsMax[2]) * 0.5);
        const Vec3d axisHead(
            (boundsMin[0] + boundsMax[0]) * 0.5,
            boundsMax[1],
            (boundsMin[2] + boundsMax[2]) * 0.5);
        const Vec3d sourceAxis = axisHead - axisBase;
        const Quatd alignAxis = (sourceAxis.norm() > 1.0e-8) ?
            Quatd::FromTwoVectors(sourceAxis.normalized(), Vec3d::UnitY()) :
            Quatd::Identity();
        const Mat4d transform =
            mat4dScale(Vec3d(scale, scale, scale)) *
            mat4dRotation(alignAxis) *
            mat4dTranslate(-axisBase);
        forcepsMesh->transform(transform, Geometry::TransformType::ApplyToData);

        toolObj->setVisualGeometry(forcepsMesh);
        toolObj->setPhysicsToVisualMap(std::make_shared<IsometricMap>(toolGeom, forcepsMesh));

        std::shared_ptr<RenderMaterial> material = toolObj->getVisualModel(0)->getRenderMaterial();
        material->setDisplayMode(RenderMaterial::DisplayMode::Surface);
        material->setDiffuseColor(Color(0.74, 0.76, 0.78));
        material->setShadingModel(RenderMaterial::ShadingModel::PBR);
        material->setRoughness(0.28);
        material->setMetalness(0.75);
        material->setIsDynamicMesh(true);
        material->setBackFaceCulling(false);
    }
    else
    {
        std::cout << "PBDMeniscusHapticSuture: failed to load forceps model "
                  << FORCEPS_TOOL_PATH << ", using line visual." << std::endl;
        toolObj->setVisualGeometry(toolGeom);
    }

    auto rayMaterial = std::make_shared<RenderMaterial>();
    rayMaterial->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);
    rayMaterial->setColor(Color(0.05, 1.0, 0.25));
    rayMaterial->setLineWidth(4.0);
    imstkNew<VisualModel> rayVisual;
    rayVisual->setGeometry(toolGeom);
    rayVisual->setRenderMaterial(rayMaterial);
    toolObj->addVisualModel(rayVisual);

    toolObj->getPbdBody()->setRigid(
        initPos,
        2.0,
        Quatd::Identity(),
        Mat3d::Identity() * 0.05);

    auto controller = toolObj->addComponent<PbdObjectController>();
    controller->setControlledObject(toolObj);
    controller->setLinearKs(10000.0);
    controller->setAngularKs(100000.0);
    controller->setForceScaling(0.0);
    controller->setSmoothingKernelSize(15);
    controller->setUseForceSmoothening(true);
    controller->setHapticOffset(Vec3d::Zero());

    return toolObj;
}

static Vec3d
getToolTipPosition(const std::shared_ptr<PbdObject> toolObj)
{
    if (toolObj == nullptr)
    {
        return Vec3d::Zero();
    }

    auto toolGeom = std::dynamic_pointer_cast<LineMesh>(toolObj->getCollidingGeometry());
    if (toolGeom == nullptr || toolGeom->getNumVertices() == 0)
    {
        return toolObj->getPbdBody()->getRigidPosition();
    }
    return toolGeom->getVertexPosition(0);
}

static bool
beginFragmentGrab(FragmentGrabState& grabState,
                  const Vec3d& toolTip,
                  const std::shared_ptr<PbdBody> body,
                  const std::shared_ptr<TetrahedralMesh> tetMesh,
                  const FracturePath& fracturePath)
{
    if (body == nullptr || body->vertices == nullptr || body->invMasses == nullptr || tetMesh == nullptr)
    {
        return false;
    }

    const FracturePathSample toolSample = fracturePath.closestSample(toolTip);
    if (toolSample.distanceToPath > fracturePath.tearRadius * 5.0)
    {
        std::cout << "PBDMeniscusHapticSuture: forceps missed fracture line; distance "
                  << toolSample.distanceToPath << "." << std::endl;
        return false;
    }

    grabState = FragmentGrabState();
    grabState.active = true;
    grabState.selectedSideSign = (toolSample.signedSideDistance >= 0.0) ? 1.0 : -1.0;
    grabState.startToolPosition = toolTip;

    const VecDataArray<double, 3>& restPositions = *tetMesh->getVertexPositions();
    const VecDataArray<double, 3>& currentPositions = *body->vertices;
    for (int i = 0; i < restPositions.size(); i++)
    {
        if ((*body->invMasses)[i] == 0.0)
        {
            continue;
        }

        const FracturePathSample sample = fracturePath.closestSample(restPositions[i]);
        if (sample.signedSideDistance * grabState.selectedSideSign < -fracturePath.tearRadius * 0.25
            || sample.distanceToPath > fracturePath.tearRadius * 8.0)
        {
            continue;
        }

        const double distanceWeight =
            1.0 - std::max(0.0, std::min(1.0, sample.distanceToPath / (fracturePath.tearRadius * 8.0)));
        grabState.nodeIds.push_back(i);
        grabState.startNodePositions.push_back(currentPositions[i]);
        grabState.weights.push_back(0.25 + 0.75 * distanceWeight);
    }

    std::cout << "PBDMeniscusHapticSuture: forceps grabbed "
              << grabState.nodeIds.size() << " fracture-side vertices." << std::endl;
    return !grabState.nodeIds.empty();
}

static double
applyFragmentGrab(const FragmentGrabState& grabState,
                  const Vec3d& toolTip,
                  const std::shared_ptr<PbdBody> body,
                  const double dt)
{
    if (!grabState.active || body == nullptr || body->vertices == nullptr
        || body->velocities == nullptr || body->invMasses == nullptr)
    {
        return 0.0;
    }

    const Vec3d toolDelta = toolTip - grabState.startToolPosition;
    const double safeDt = std::max(dt, 1.0e-6);
    for (size_t i = 0; i < grabState.nodeIds.size(); i++)
    {
        const int nodeId = grabState.nodeIds[i];
        if ((*body->invMasses)[nodeId] == 0.0)
        {
            continue;
        }

        const Vec3d current = (*body->vertices)[nodeId];
        const Vec3d desired = grabState.startNodePositions[i] + toolDelta * grabState.weights[i];
        const Vec3d next = current + (desired - current) * 0.45;
        (*body->vertices)[nodeId] = next;
        (*body->velocities)[nodeId] = (next - current) / safeDt;
    }
    return toolDelta.norm();
}

static void
translateTissueByDelta(const MeniscusTissue& tissue, const Vec3d& delta)
{
    if (tissue.tetMesh == nullptr || tissue.object == nullptr)
    {
        return;
    }

    VecDataArray<double, 3>& vertices = *tissue.tetMesh->getVertexPositions();
    for (int i = 0; i < vertices.size(); i++)
    {
        vertices[i] += delta;
    }
    vertices.postModified();
    tissue.tetMesh->postModified();
    if (tissue.surfaceMesh != nullptr)
    {
        tissue.surfaceMesh->getVertexPositions()->postModified();
        tissue.surfaceMesh->computeVertexNormals();
        tissue.surfaceMesh->postModified();
    }
    if (tissue.edgeMesh != nullptr)
    {
        tissue.edgeMesh->getVertexPositions()->postModified();
        tissue.edgeMesh->postModified();
    }
}

static std::shared_ptr<PbdObject>
makeNeedleObj(const std::shared_ptr<PbdModel> model, const Vec3d& initPos)
{
    auto needleObj = std::make_shared<PbdObject>("sutureNeedle");

    std::shared_ptr<SurfaceMesh> needleMesh =
        MeshIO::read<SurfaceMesh>(std::string(DEMO_IMSTK_DATA_ROOT)
            + "/Surgical Instruments/Needles/c6_suture.stl");
    std::shared_ptr<LineMesh> needleLineMesh =
        MeshIO::read<LineMesh>(std::string(DEMO_IMSTK_DATA_ROOT)
            + "/Surgical Instruments/Needles/c6_suture_hull.vtk");
    if (needleLineMesh == nullptr)
    {
        auto vertices = std::make_shared<VecDataArray<double, 3>>(2);
        (*vertices)[0] = Vec3d(-0.02, 0.0, 0.0);
        (*vertices)[1] = Vec3d(0.02, 0.0, 0.0);
        auto indices = std::make_shared<VecDataArray<int, 2>>(1);
        (*indices)[0] = Vec2i(0, 1);
        needleLineMesh = std::make_shared<LineMesh>();
        needleLineMesh->initialize(vertices, indices);
    }
    if (needleMesh != nullptr)
    {
        needleMesh->translate(Vec3d(0.0, -0.0047, -0.0087), Geometry::TransformType::ApplyToData);
        needleLineMesh->translate(Vec3d(0.0, -0.0047, -0.0087), Geometry::TransformType::ApplyToData);
        needleMesh->scale(NEEDLE_SCALE, Geometry::TransformType::ApplyToData);
        needleLineMesh->scale(NEEDLE_SCALE, Geometry::TransformType::ApplyToData);
        increaseNeedleCurvature(needleMesh->getVertexPositions());
        increaseNeedleCurvature(needleLineMesh->getVertexPositions());
        needleMesh->postModified();
        needleLineMesh->postModified();
        needleObj->setVisualGeometry(needleMesh);
        needleObj->setPhysicsToVisualMap(std::make_shared<IsometricMap>(needleLineMesh, needleMesh));
        needleObj->getVisualModel(0)->getRenderMaterial()->setColor(Color::Orange);
        needleObj->getVisualModel(0)->getRenderMaterial()->setBackFaceCulling(false);

        auto needleHullMaterial = std::make_shared<RenderMaterial>();
        needleHullMaterial->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);
        needleHullMaterial->setColor(Color::Yellow);
        needleHullMaterial->setLineWidth(4.0);
        imstkNew<VisualModel> needleHullVisual;
        needleHullVisual->setGeometry(needleLineMesh);
        needleHullVisual->setRenderMaterial(needleHullMaterial);
        needleObj->addVisualModel(needleHullVisual);
    }
    else
    {
        needleObj->setVisualGeometry(needleLineMesh);
    }
    needleObj->setCollidingGeometry(needleLineMesh);
    needleObj->setPhysicsGeometry(needleLineMesh);
    needleObj->setDynamicalModel(model);
    needleObj->getPbdBody()->setRigid(initPos, 1.0, Quatd::Identity(), Mat3d::Identity() * 0.01);

    return needleObj;
}

static std::shared_ptr<PbdObject>
makePbdString(const std::string& name,
              const Vec3d& pos,
              const Vec3d& dir,
              const int numVerts,
              const double stringLength,
              const std::shared_ptr<PbdObject> needleObj)
{
    auto stringObj = std::make_shared<PbdObject>(name);
    std::shared_ptr<LineMesh> stringMesh =
        GeometryUtils::toLineGrid(pos, dir, stringLength, numVerts);

    auto material = std::make_shared<RenderMaterial>();
    material->setBackFaceCulling(false);
    material->setColor(Color::Red);
    material->setLineWidth(2.0);
    material->setPointSize(6.0);
    material->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);

    stringObj->setVisualGeometry(stringMesh);
    stringObj->getVisualModel(0)->setRenderMaterial(material);
    stringObj->setPhysicsGeometry(stringMesh);
    stringObj->setCollidingGeometry(stringMesh);

    std::shared_ptr<PbdModel> model = needleObj->getPbdModel();
    stringObj->setDynamicalModel(model);
    stringObj->getPbdBody()->uniformMassValue = 0.02;

    const int bodyHandle = stringObj->getPbdBody()->bodyHandle;
    model->getConfig()->enableConstraint(
        PbdModelConfig::ConstraintGenType::Distance,
        1000.0,
        bodyHandle);
    model->getConfig()->enableBendConstraint(1.0, 1, true, bodyHandle);

    auto needleLineMesh = std::dynamic_pointer_cast<LineMesh>(needleObj->getPhysicsGeometry());
    model->getConfig()->addPbdConstraintFunctor([=](PbdConstraintContainer& container)
        {
            const Vec3d endOfNeedle = (*needleLineMesh->getVertexPositions())[0];
            auto attachmentConstraint = std::make_shared<PbdBodyToBodyDistanceConstraint>();
            attachmentConstraint->initConstraint(
                model->getBodies(),
                { needleObj->getPbdBody()->bodyHandle, 0 },
                endOfNeedle,
                { stringObj->getPbdBody()->bodyHandle, 0 },
                0.0,
                0.0000001);
            container.addConstraint(attachmentConstraint);
        });

    return stringObj;
}

int
main()
{
    Logger::startLogger();

    imstkNew<Scene> scene("PBDMeniscusHapticSuture");
    scene->getActiveCamera()->setPosition(0.0, 4.0, 11.0);
    scene->getActiveCamera()->setFocalPoint(0.0, 0.0, 0.0);
    scene->getActiveCamera()->setViewUp(0.0, 1.0, 0.0);

    imstkNew<PbdModel> pbdModel;
    pbdModel->getConfig()->m_doPartitioning = true;
    pbdModel->getConfig()->m_gravity = Vec3d::Zero();
    pbdModel->getConfig()->m_dt = PBD_TIME_STEP;
    pbdModel->getConfig()->m_iterations = PBD_SOLVER_ITERATIONS;
    pbdModel->getConfig()->m_solverType = PbdConstraint::SolverType::xPBD;

    MeniscusTissue meniscusTissue = makeImportedMeniscusObject(pbdModel);
    if (meniscusTissue.object == nullptr
        || meniscusTissue.tetMesh == nullptr
        || meniscusTissue.surfaceMesh == nullptr)
    {
        std::cout << "PBDMeniscusHapticSuture: meniscus tissue setup failed." << std::endl;
        return 1;
    }
    scene->addSceneObject(meniscusTissue.object);

    imstkNew<SceneObject> edgeOverlayObj("Imported meniscus boundary edge overlay");
    if (SHOW_MENISCUS_EDGE_OVERLAY && meniscusTissue.edgeMesh != nullptr)
    {
        auto edgeMaterial = std::make_shared<RenderMaterial>();
        edgeMaterial->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);
        edgeMaterial->setColor(Color(0.08, 0.18, 0.22));
        edgeMaterial->setLineWidth(1.0);

        imstkNew<VisualModel> edgeVisual;
        edgeVisual->setGeometry(meniscusTissue.edgeMesh);
        edgeVisual->setRenderMaterial(edgeMaterial);
        edgeOverlayObj->addVisualModel(edgeVisual);
        scene->addSceneObject(edgeOverlayObj);
    }

    DemoStage demoStage = DemoStage::ForcepsTear;
    bool fractureComplete = false;
    bool sutureStageActivated = false;
    FragmentGrabState fragmentGrabState;
    double fracturePullDistance = 0.0;
    FracturePath fracturePath = makeFracturePathFromTissue(meniscusTissue.tetMesh);

    auto makeRouteLine = [](const std::array<Vec3d, 3>& points) -> std::shared_ptr<LineMesh>
        {
            auto vertices = std::make_shared<VecDataArray<double, 3>>(3);
            auto edges = std::make_shared<VecDataArray<int, 2>>(2);
            for (int i = 0; i < 3; i++)
            {
                (*vertices)[i] = points[i];
            }
            (*edges)[0] = Vec2i(0, 1);
            (*edges)[1] = Vec2i(1, 2);
            auto line = std::make_shared<LineMesh>();
            line->initialize(vertices, edges);
            return line;
        };
    auto makeRouteLineFromVertexIds =
        [](const std::shared_ptr<TetrahedralMesh> tetMesh,
           const auto& ids) -> std::shared_ptr<LineMesh>
        {
            auto vertices = std::make_shared<VecDataArray<double, 3>>(
                static_cast<int>(ids.size()));
            auto edges = std::make_shared<VecDataArray<int, 2>>();
            for (int i = 0; i < static_cast<int>(ids.size()); i++)
            {
                (*vertices)[i] = tetMesh->getVertexPosition(ids[i]);
                if (i > 0)
                {
                    edges->push_back(Vec2i(i - 1, i));
                }
            }
            auto line = std::make_shared<LineMesh>();
            line->initialize(vertices, edges);
            return line;
        };

    imstkNew<SceneObject> fractureRouteObj("Meniscus fracture route");
    auto addRouteVisual =
        [&](const std::shared_ptr<LineMesh> line, const Color& color, const double width)
        {
            auto material = std::make_shared<RenderMaterial>();
            material->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);
            material->setColor(color);
            material->setLineWidth(width);
            imstkNew<VisualModel> visual;
            visual->setGeometry(line);
            visual->setRenderMaterial(material);
            fractureRouteObj->addVisualModel(visual);
        };

    std::shared_ptr<LineMesh> leftLipLine = makeRouteLine(fracturePath.leftLip);
    std::shared_ptr<LineMesh> rightLipLine = makeRouteLine(fracturePath.rightLip);
    std::shared_ptr<LineMesh> centerTearLine = makeRouteLine(fracturePath.center);
    std::shared_ptr<LineMesh> notchLine = makeRouteLineFromVertexIds(
        meniscusTissue.tetMesh,
        MENISCUS_FRAGMENT_NOTCH_DISPLAY_VERTEX_IDS);
    std::shared_ptr<LineMesh> oppositeNotchLine = makeRouteLineFromVertexIds(
        meniscusTissue.tetMesh,
        MENISCUS_FRAGMENT_OPPOSITE_NOTCH_VERTEX_IDS);
    std::shared_ptr<LineMesh> explicitFragmentLineA = makeRouteLineFromVertexIds(
        meniscusTissue.tetMesh,
        MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_A);
    std::shared_ptr<LineMesh> explicitFragmentLineB = makeRouteLineFromVertexIds(
        meniscusTissue.tetMesh,
        MENISCUS_FRAGMENT_EXPLICIT_VERTEX_IDS_B);
    addRouteVisual(leftLipLine, Color(0.0, 1.0, 1.0), 6.0);
    addRouteVisual(rightLipLine, Color(1.0, 0.55, 0.05), 6.0);
    addRouteVisual(centerTearLine, Color::Red, 8.0);
    addRouteVisual(notchLine, Color(0.75, 0.2, 1.0), 6.0);
    addRouteVisual(oppositeNotchLine, Color(0.1, 1.0, 0.25), 6.0);
    addRouteVisual(explicitFragmentLineA, Color(0.1, 0.45, 1.0), 5.0);
    addRouteVisual(explicitFragmentLineB, Color(0.95, 0.95, 0.95), 5.0);
    auto endpointMaterial = std::make_shared<RenderMaterial>();
    endpointMaterial->setColor(Color::Yellow);
    endpointMaterial->setShadingModel(RenderMaterial::ShadingModel::Phong);
    for (int i = 0; i < 3; i++)
    {
        auto sphere = std::make_shared<Sphere>(fracturePath.center[i], 0.035);
        imstkNew<VisualModel> endpointVisual;
        endpointVisual->setGeometry(sphere);
        endpointVisual->setRenderMaterial(endpointMaterial);
        fractureRouteObj->addVisualModel(endpointVisual);
    }

    const Vec3d tissueCenter = (meniscusTissue.boundsMin + meniscusTissue.boundsMax) * 0.5;
    const Vec3d hapticWorkspaceOffset = tissueCenter + Vec3d(0.0, 0.75, -0.25);
    const Vec3d leftToolStart = hapticWorkspaceOffset + Vec3d(0.0, 0.0, 0.6);
    std::shared_ptr<PbdObject> forcepsToolObj = makeForcepsToolObj(pbdModel, leftToolStart);
    scene->addSceneObject(forcepsToolObj);

    std::shared_ptr<PbdObject> leftToolObj;
    std::shared_ptr<PbdObject> needleObj;
    std::shared_ptr<PbdObject> sutureThreadObj;
    std::shared_ptr<PbdObjectGrasping> leftNeedleGrasping;
    std::shared_ptr<PbdObjectGrasping> leftThreadGrasping;
    MeniscusTissue sutureMeniscusTissue;
    MeniscusTissue fragmentTissue;
    std::vector<std::shared_ptr<PbdObjectCollision>> tissueSutureCollisions;
    std::vector<std::shared_ptr<PbdObjectCollision>> commonSutureCollisions;
    Vec3d fragmentFollowStartToolTip = Vec3d::Zero();

    FragmentSplit fragmentSplit = buildSeededFragmentSplit(meniscusTissue, fracturePath);
    if (fragmentSplit.mainMesh != nullptr && fragmentSplit.fragmentMesh != nullptr)
    {
        sutureMeniscusTissue = makeMeniscusObjectFromMesh(
            pbdModel,
            fragmentSplit.mainMesh,
            "Fracture-excluded main meniscus PBD tissue");
        fragmentTissue = makeMeniscusObjectFromMesh(
            pbdModel,
            fragmentSplit.fragmentMesh,
            "Detached notch-fragment meniscus PBD tissue");
        applyOriginalBoundarySurface(
            sutureMeniscusTissue,
            meniscusTissue.tetMesh,
            fragmentSplit.mainTetIds,
            "fracture-excluded main body");
        applyOriginalBoundarySurface(
            fragmentTissue,
            meniscusTissue.tetMesh,
            fragmentSplit.fragmentTetIds,
            "detached notch fragment");
        scene->addSceneObject(sutureMeniscusTissue.object);
        scene->addSceneObject(fragmentTissue.object);
        hideSceneObjectVisuals(sutureMeniscusTissue.object);
        hideSceneObjectVisuals(fragmentTissue.object);
        if (fragmentTissue.object != nullptr)
        {
            fragmentTissue.object->getPbdBody()->fixedNodeIds.clear();
        }

        const Vec3d sutureTissueCenter =
            (meniscusTissue.boundsMin + meniscusTissue.boundsMax) * 0.5;
        leftToolObj = makeLapToolObj("leftHapticLapTool", pbdModel, leftToolStart);
        scene->addSceneObject(leftToolObj);
        hideSceneObjectVisuals(leftToolObj);

        needleObj = makeNeedleObj(pbdModel, sutureTissueCenter + Vec3d(0.25, 0.8, 0.8));
        scene->addSceneObject(needleObj);
        hideSceneObjectVisuals(needleObj);

        sutureThreadObj = makePbdString(
            "sutureThread",
            sutureTissueCenter + Vec3d(0.25, 0.8, 0.8),
            Vec3d(0.0, 0.0, 1.0),
            80,
            3.7,
            needleObj);
        scene->addSceneObject(sutureThreadObj);
        hideSceneObjectVisuals(sutureThreadObj);

        if (ENABLE_TOOL_NEEDLE_CONTACT)
        {
            auto leftNeedleCollision = std::make_shared<PbdObjectCollision>(leftToolObj, needleObj);
            leftNeedleCollision->setRigidBodyCompliance(0.0001);
            leftNeedleCollision->setUseCorrectVelocity(false);
            scene->addInteraction(leftNeedleCollision);
        }

        if (ENABLE_TOOL_THREAD_CONTACT)
        {
            auto leftThreadCollision = std::make_shared<PbdObjectCollision>(leftToolObj, sutureThreadObj);
            leftThreadCollision->setRigidBodyCompliance(0.0001);
            leftThreadCollision->setUseCorrectVelocity(false);
            leftThreadCollision->setEnabled(false);
            scene->addInteraction(leftThreadCollision);
            commonSutureCollisions.push_back(leftThreadCollision);
        }

        if (ENABLE_NEEDLE_TISSUE_CONTACT)
        {
            auto needleTissueCollision = std::make_shared<PbdObjectCollision>(needleObj, sutureMeniscusTissue.object);
            needleTissueCollision->setRigidBodyCompliance(0.00005);
            needleTissueCollision->setUseCorrectVelocity(false);
            needleTissueCollision->setEnabled(false);
            scene->addInteraction(needleTissueCollision);
            tissueSutureCollisions.push_back(needleTissueCollision);
        }
        if (ENABLE_THREAD_TISSUE_CONTACT)
        {
            auto threadTissueCollision = std::make_shared<PbdObjectCollision>(sutureThreadObj, sutureMeniscusTissue.object);
            threadTissueCollision->setDeformableStiffnessA(0.05);
            threadTissueCollision->setDeformableStiffnessB(0.05);
            threadTissueCollision->setUseCorrectVelocity(false);
            threadTissueCollision->setEnabled(false);
            scene->addInteraction(threadTissueCollision);
            tissueSutureCollisions.push_back(threadTissueCollision);
        }
        if (ENABLE_THREAD_SELF_COLLISION)
        {
            auto threadSelfCollision = std::make_shared<PbdObjectCollision>(sutureThreadObj, sutureThreadObj);
            threadSelfCollision->setDeformableStiffnessA(0.05);
            threadSelfCollision->setDeformableStiffnessB(0.05);
            threadSelfCollision->setEnabled(false);
            scene->addInteraction(threadSelfCollision);
            commonSutureCollisions.push_back(threadSelfCollision);
        }

        leftNeedleGrasping = std::make_shared<PbdObjectGrasping>(needleObj, leftToolObj);
        leftNeedleGrasping->setCompliance(0.00001);
        scene->addInteraction(leftNeedleGrasping);
        leftThreadGrasping = std::make_shared<PbdObjectGrasping>(sutureThreadObj, leftToolObj);
        leftThreadGrasping->setCompliance(0.00001);
        scene->addInteraction(leftThreadGrasping);
    }
    else
    {
        std::cout << "PBDMeniscusHapticSuture: failed to prebuild fracture-excluded main body." << std::endl;
    }

    auto nearestPointOnSegment = [](const Vec3d& p, const Vec3d& a, const Vec3d& b) -> Vec3d
        {
            const Vec3d ab = b - a;
            const double denom = ab.squaredNorm();
            if (denom < 1.0e-12)
            {
                return a;
            }
            const double t = std::max(0.0, std::min(1.0, (p - a).dot(ab) / denom));
            return Vec3d(a + t * ab);
        };

    auto capsuleSignedDistance = [&](const std::shared_ptr<Capsule> capsule, const Vec3d& p)
        {
            const Vec3d center = capsule->getPosition();
            const Vec3d axis = capsule->getOrientation().toRotationMatrix().col(1).normalized();
            const double halfLength = capsule->getLength() * 0.5;
            const Vec3d a = center - axis * halfLength;
            const Vec3d b = center + axis * halfLength;
            return (p - nearestPointOnSegment(p, a, b)).norm() - capsule->getRadius();
        };

    std::vector<std::shared_ptr<PbdBodyToBodyDistanceConstraint>> leftManualNeedleConstraints;
    std::vector<PbdConstraint*> leftManualNeedleConstraintPtrs;
    bool needleRigidFollowActive = false;
    Vec3d needleRigidFollowLocalPosition = Vec3d::Zero();
    Quatd needleRigidFollowLocalOrientation = Quatd::Identity();

    auto clearManualNeedleGrasp =
        [](std::vector<std::shared_ptr<PbdBodyToBodyDistanceConstraint>>& constraints,
           std::vector<PbdConstraint*>& constraintPtrs)
        {
            constraints.clear();
            constraintPtrs.clear();
        };

    auto beginNeedleRigidFollow =
        [&](const std::shared_ptr<PbdObject> toolObj)
        {
            if (toolObj == nullptr || needleObj == nullptr)
            {
                return;
            }
            const auto toolBody = toolObj->getPbdBody();
            const auto needleBody = needleObj->getPbdBody();
            const Vec3d toolPos = toolBody->getRigidPosition();
            const Quatd toolOrientation = toolBody->getRigidOrientation();
            const Vec3d needlePos = needleBody->getRigidPosition();
            const Quatd needleOrientation = needleBody->getRigidOrientation();

            needleRigidFollowLocalPosition =
                toolOrientation.inverse()._transformVector(needlePos - toolPos);
            needleRigidFollowLocalOrientation =
                toolOrientation.inverse() * needleOrientation;
            needleRigidFollowActive = true;
        };

    auto endNeedleRigidFollow =
        [&]()
        {
            needleRigidFollowActive = false;
        };

    auto syncNeedleRigidFollow =
        [&](const std::shared_ptr<PbdObject> toolObj)
        {
            if (!needleRigidFollowActive || toolObj == nullptr || needleObj == nullptr)
            {
                return;
            }

            const auto toolBody = toolObj->getPbdBody();
            const auto needleBody = needleObj->getPbdBody();
            const Vec3d toolPos = toolBody->getRigidPosition();
            const Quatd toolOrientation = toolBody->getRigidOrientation();
            const Vec3d targetPos =
                toolPos + toolOrientation._transformVector(needleRigidFollowLocalPosition);
            const Quatd targetOrientation =
                toolOrientation * needleRigidFollowLocalOrientation;

            needleBody->overrideRigidPositionAndOrientation(targetPos, targetOrientation);
            if (needleBody->velocities != nullptr && needleBody->angularVelocities != nullptr)
            {
                needleBody->overrideLinearAndAngularVelocity(Vec3d::Zero(), Vec3d::Zero());
            }
        };

    auto tryBeginManualNeedleGrasp =
        [&](const std::string& label,
            const std::shared_ptr<PbdObject> toolObj,
            std::vector<std::shared_ptr<PbdBodyToBodyDistanceConstraint>>& constraints,
            std::vector<PbdConstraint*>& constraintPtrs)
        {
            if (!constraints.empty())
            {
                return true;
            }
            if (toolObj == nullptr || needleObj == nullptr)
            {
                return false;
            }

            auto graspCapsule = std::dynamic_pointer_cast<Capsule>(
                toolObj->getVisualModel(1)->getGeometry());
            auto needleLineMesh = std::dynamic_pointer_cast<LineMesh>(needleObj->getPhysicsGeometry());
            if (graspCapsule == nullptr || needleLineMesh == nullptr || needleLineMesh->getNumVertices() == 0)
            {
                return false;
            }

            const VecDataArray<double, 3>& needleVertices = *needleLineMesh->getVertexPositions();
            int closestVertex = -1;
            double closestSignedDistance = std::numeric_limits<double>::max();
            for (int i = 0; i < needleVertices.size(); i++)
            {
                const double signedDistance = capsuleSignedDistance(graspCapsule, needleVertices[i]);
                if (signedDistance < closestSignedDistance)
                {
                    closestSignedDistance = signedDistance;
                    closestVertex = i;
                }
            }

            std::cout << "PBDMeniscusHapticSuture: " << label
                      << " manual needle grasp nearest distance "
                      << closestSignedDistance << std::endl;

            if (closestVertex < 0 || closestSignedDistance > MANUAL_NEEDLE_GRASP_TOLERANCE)
            {
                return false;
            }

            std::vector<int> needleLockVertices = { closestVertex };
            const int tipVertex = static_cast<int>(needleVertices.size()) - 1;
            if (tipVertex != closestVertex)
            {
                needleLockVertices.push_back(tipVertex);
            }
            if (closestVertex != 0 && tipVertex != 0)
            {
                needleLockVertices.push_back(0);
            }

            for (const int vertexId : needleLockVertices)
            {
                const Vec3d& lockPt = needleVertices[vertexId];
                auto constraint = std::make_shared<PbdBodyToBodyDistanceConstraint>();
                constraint->initConstraint(
                    pbdModel->getBodies(),
                    { toolObj->getPbdBody()->bodyHandle, 0 },
                    lockPt,
                    { needleObj->getPbdBody()->bodyHandle, 0 },
                    lockPt,
                    0.0,
                    0.000001);
                constraints.push_back(constraint);
                constraintPtrs.push_back(constraint.get());
            }

            std::cout << "PBDMeniscusHapticSuture: " << label
                      << " manual needle grasp active with "
                      << constraints.size() << " constraints." << std::endl;
            return true;
        };

    std::vector<std::shared_ptr<PbdBaryPointToPointConstraint>> sutureAnchorConstraints;
    std::set<int> anchoredThreadVertices;
    bool havePreviousNeedleTipHit = false;
    double previousNeedleTipSignedDistance = 0.0;
    int anchorCooldownFrames = 0;

    auto addSutureAnchor = [&](const SurfaceHit& hit)
        {
            if (!hit.valid || static_cast<int>(sutureAnchorConstraints.size()) >= MAX_SUTURE_ANCHORS)
            {
                return;
            }
            if (sutureThreadObj == nullptr || meniscusTissue.object == nullptr)
            {
                return;
            }

            auto threadMesh = std::dynamic_pointer_cast<LineMesh>(sutureThreadObj->getPhysicsGeometry());
            if (threadMesh == nullptr || threadMesh->getNumVertices() < 2)
            {
                return;
            }

            const VecDataArray<double, 3>& threadVertices = *threadMesh->getVertexPositions();
            int closestThreadVertex = -1;
            double closestThreadDistance = std::numeric_limits<double>::max();
            for (int i = 1; i < threadVertices.size(); i++)
            {
                if (anchoredThreadVertices.count(i) > 0)
                {
                    continue;
                }

                const double distance = (threadVertices[i] - hit.point).squaredNorm();
                if (distance < closestThreadDistance)
                {
                    closestThreadDistance = distance;
                    closestThreadVertex = i;
                }
            }
            if (closestThreadVertex < 0)
            {
                return;
            }

            auto constraint = std::make_shared<PbdBaryPointToPointConstraint>();
            constraint->initConstraint(
                { PbdParticleId(sutureThreadObj->getPbdBody()->bodyHandle, closestThreadVertex) },
                { 1.0 },
                {
                    PbdParticleId(meniscusTissue.object->getPbdBody()->bodyHandle, hit.triangle[0]),
                    PbdParticleId(meniscusTissue.object->getPbdBody()->bodyHandle, hit.triangle[1]),
                    PbdParticleId(meniscusTissue.object->getPbdBody()->bodyHandle, hit.triangle[2])
                },
                { hit.barycentric[0], hit.barycentric[1], hit.barycentric[2] },
                0.85,
                0.85);
            pbdModel->getConstraints()->addConstraint(constraint);
            sutureAnchorConstraints.push_back(constraint);
            anchoredThreadVertices.insert(closestThreadVertex);
            anchorCooldownFrames = PUNCTURE_ANCHOR_COOLDOWN_FRAMES;

            std::cout << "PBDMeniscusHapticSuture: added suture anchor "
                      << sutureAnchorConstraints.size()
                      << " at tissue triangle " << hit.triangleId
                      << " using thread vertex " << closestThreadVertex << std::endl;
        };

    auto activateSutureStage = [&]()
        {
            if (sutureStageActivated)
            {
                return;
            }

            if (sutureMeniscusTissue.object == nullptr || leftToolObj == nullptr
                || needleObj == nullptr || sutureThreadObj == nullptr)
            {
                std::cout << "PBDMeniscusHapticSuture: suture stage was not prebuilt." << std::endl;
                return;
            }

            hideSceneObjectVisuals(meniscusTissue.object);
            hideSceneObjectVisuals(fragmentTissue.object);
            hideSceneObjectVisuals(forcepsToolObj);
            hideSceneObjectVisuals(fractureRouteObj);
            hideSceneObjectVisuals(sutureMeniscusTissue.object);

            for (const auto& collision : commonSutureCollisions)
            {
                collision->setEnabled(true);
            }
            for (const auto& collision : tissueSutureCollisions)
            {
                collision->setEnabled(true);
            }
            meniscusTissue = sutureMeniscusTissue;
            showSceneObjectVisuals(meniscusTissue.object);
            showSceneObjectVisuals(leftToolObj);
            showSceneObjectVisuals(needleObj);
            showSceneObjectVisuals(sutureThreadObj);

            auto forcepsController = forcepsToolObj->getComponent<PbdObjectController>();
            if (forcepsController != nullptr)
            {
                forcepsController->setDevice(nullptr);
            }
            sutureStageActivated = true;
            demoStage = DemoStage::Suture;
            std::cout << "PBDMeniscusHapticSuture: switched to suture stage with notch-fragment excluded main body." << std::endl;
        };

    imstkNew<DirectionalLight> light;
    light->setFocalPoint(Vec3d(-1.0, -1.0, -1.0));
    light->setIntensity(1.0);
    scene->addLight("Light", light);

    imstkNew<VTKViewer> viewer;
    viewer->setActiveScene(scene);
    viewer->setInfoLevel(0);
    viewer->setWindowTitle("PBD Meniscus Haptic Suture");

    imstkNew<SceneManager> sceneManager;
    sceneManager->setActiveScene(scene);

    imstkNew<SimulationManager> driver;
    driver->addModule(viewer);
    driver->addModule(sceneManager);
    driver->setDesiredDt(DRIVER_TIME_STEP);

    bool leftGraspActive = false;
    bool hapticButtonsArmed = false;
    int hapticButtonReleasedFrames = 0;
    int hapticActiveButton = -1;
    int hapticLastButton = -1;
    int hapticLastButtonState = 0;

#ifdef iMSTK_USE_HAPTICS
    std::shared_ptr<DeviceManager> hapticManager = DeviceManagerFactory::makeDeviceManager();
    std::shared_ptr<DeviceClient> leftDeviceClient = hapticManager->makeDeviceClient("Default Device");
    driver->addModule(hapticManager);

    auto activeController = forcepsToolObj->getComponent<PbdObjectController>();
    activeController->setDevice(leftDeviceClient);
    activeController->setTranslationScaling(HAPTIC_TRANSLATION_SCALING);
    activeController->setTranslationOffset(hapticWorkspaceOffset);
    activeController->setUseSpring(ENABLE_HAPTIC_FORCE_FEEDBACK);
    activeController->setForceScaling(ENABLE_HAPTIC_FORCE_FEEDBACK ? 0.01 : 0.0);
    leftDeviceClient->setForceEnabled(ENABLE_HAPTIC_FORCE_FEEDBACK);
    leftDeviceClient->setForce(Vec3d::Zero());

    if (!ENABLE_HAPTIC_FORCE_FEEDBACK)
    {
        connect<Event>(sceneManager, &SceneManager::postUpdate,
            [&](Event*)
            {
                leftDeviceClient->setForce(Vec3d::Zero());
            });
    }

    connect<ButtonEvent>(leftDeviceClient, &DeviceClient::buttonStateChanged,
        [&](ButtonEvent* e)
        {
            std::cout << "PBDMeniscusHapticSuture: haptic button event id="
                      << e->m_button << " state=" << e->m_buttonState << std::endl;

            if (e->m_button < 0 || e->m_button >= HAPTIC_BUTTON_COUNT)
            {
                return;
            }
            hapticLastButton = e->m_button;
            hapticLastButtonState = e->m_buttonState;

            if (e->m_buttonState == BUTTON_PRESSED)
            {
                if (!hapticButtonsArmed)
                {
                    std::cout << "PBDMeniscusHapticSuture: ignored initial haptic button "
                              << e->m_button << " pressed before release-arm." << std::endl;
                    return;
                }
                if (leftGraspActive)
                {
                    return;
                }

                if (demoStage == DemoStage::ForcepsTear)
                {
                    const int bodyId = meniscusTissue.object->getPbdBody()->bodyHandle;
                    std::shared_ptr<PbdBody> body = pbdModel->getBody(bodyId);
                    if (beginFragmentGrab(
                            fragmentGrabState,
                            getToolTipPosition(forcepsToolObj),
                            body,
                            meniscusTissue.tetMesh,
                            fracturePath))
                    {
                        leftGraspActive = true;
                        hapticActiveButton = e->m_button;
                    }
                    return;
                }

                if (leftToolObj == nullptr || leftThreadGrasping == nullptr)
                {
                    return;
                }

                auto graspCapsule = std::dynamic_pointer_cast<Capsule>(
                    leftToolObj->getVisualModel(1)->getGeometry());
                const bool beganNeedleGrasp = tryBeginManualNeedleGrasp(
                    "left haptic",
                    leftToolObj,
                    leftManualNeedleConstraints,
                    leftManualNeedleConstraintPtrs);
                if (beganNeedleGrasp)
                {
                    beginNeedleRigidFollow(leftToolObj);
                }
                leftThreadGrasping->beginCellGrasp(graspCapsule);
                leftGraspActive = true;
                hapticActiveButton = e->m_button;
                std::cout << "PBDMeniscusHapticSuture: left haptic grasp button "
                          << e->m_button << " pressed." << std::endl;
            }
            else if (e->m_buttonState == BUTTON_RELEASED)
            {
                if (hapticActiveButton != -1 && e->m_button != hapticActiveButton)
                {
                    return;
                }
                if (demoStage == DemoStage::ForcepsTear)
                {
                    fragmentGrabState.active = false;
                    leftGraspActive = false;
                    hapticActiveButton = -1;
                    return;
                }
                clearManualNeedleGrasp(leftManualNeedleConstraints, leftManualNeedleConstraintPtrs);
                endNeedleRigidFollow();
                if (leftThreadGrasping != nullptr)
                {
                    leftThreadGrasping->endGrasp();
                }
                leftGraspActive = false;
                hapticActiveButton = -1;
                std::cout << "PBDMeniscusHapticSuture: left haptic grasp button "
                          << e->m_button << " released." << std::endl;
            }
        });

    connect<Event>(sceneManager, &SceneManager::postUpdate,
        [&](Event*)
        {
            if (hapticButtonsArmed)
            {
                return;
            }

            bool buttonsReleased = true;
            for (int buttonId = 0; buttonId < HAPTIC_BUTTON_COUNT; buttonId++)
            {
                if (leftDeviceClient->getButton(buttonId) == BUTTON_PRESSED)
                {
                    buttonsReleased = false;
                    hapticLastButton = buttonId;
                    hapticLastButtonState = BUTTON_PRESSED;
                    break;
                }
            }
            hapticButtonReleasedFrames = buttonsReleased ? hapticButtonReleasedFrames + 1 : 0;
            if (hapticButtonReleasedFrames >= HAPTIC_BUTTON_ARM_RELEASE_FRAMES)
            {
                hapticButtonsArmed = true;
                std::cout << "PBDMeniscusHapticSuture: haptic grasp buttons armed after startup release." << std::endl;
            }
        });
#else
    hapticButtonsArmed = true;
    std::cout << "PBDMeniscusHapticSuture: haptics are not compiled; the left tool is stationary." << std::endl;
#endif

    double latestSceneDt = driver->getDesiredDt();

    scene->addSceneObject(SimulationUtils::createDefaultSceneControl(driver));

    imstkNew<SceneObject> diagnosticsObj("PBDMeniscusHapticSuture diagnostics");
    auto diagnosticsText = diagnosticsObj->addComponent<TextVisualModel>("PBDMeniscusHapticSutureDiagnosticsText");
    diagnosticsText->setPosition(TextVisualModel::DisplayPosition::LowerLeft);
    diagnosticsText->setFontSize(24.0);
    diagnosticsText->setTextColor(Color::White);
    diagnosticsText->setText("FPS: -- | sim dt: -- ms | anchors: -- | arm: -- | raw: -- | active: -- | btn: -- | LN/LT: --");

    double visualDtAccum = 0.0;
    int visualFrameCount = 0;
    auto diagnosticsUpdate = diagnosticsObj->addComponent<LambdaBehaviour>("PBDMeniscusHapticSutureDiagnosticsUpdate");
    diagnosticsUpdate->setVisualUpdate([&, diagnosticsText](const double& visualDt)
        {
            visualDtAccum += visualDt;
            visualFrameCount++;
            if (visualDtAccum < 0.25)
            {
                return;
            }

            const double visualFps = static_cast<double>(visualFrameCount) / visualDtAccum;
            const double tearPercent = fracturePath.length > 0.0 && fractureComplete ? 100.0 : 0.0;
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(1)
                   << "FPS: " << visualFps
                   << " | sim dt: " << latestSceneDt * 1000.0 << " ms"
                   << " | stage: " << (demoStage == DemoStage::ForcepsTear ? "tear" : "suture")
                   << " | pull: " << fracturePullDistance
                   << " | tear: " << tearPercent << "%"
                   << " | anchors: " << sutureAnchorConstraints.size()
                   << " | arm: " << (hapticButtonsArmed ? "1" : "0")
                   << " | raw: " << hapticLastButton << "/" << hapticLastButtonState
                   << " | active: " << hapticActiveButton
                   << " | btn: " << (leftGraspActive ? "L" : "-")
                   << " | LN/LT: "
                   << (!leftManualNeedleConstraints.empty() ? "1" : "0")
                   << (leftThreadGrasping != nullptr && leftThreadGrasping->hasConstraints() ? "1" : "0");
            diagnosticsText->setText(stream.str());

            visualDtAccum = 0.0;
            visualFrameCount = 0;
        });
    scene->addSceneObject(diagnosticsObj);

    connect<Event>(sceneManager, &SceneManager::preUpdate,
        [&](Event*)
        {
            latestSceneDt = sceneManager->getDt();
        });

    connect<Event>(sceneManager, &SceneManager::postUpdate,
        [&](Event*)
        {
            if (demoStage == DemoStage::ForcepsTear)
            {
                if (fragmentGrabState.active)
                {
                    const int bodyId = meniscusTissue.object->getPbdBody()->bodyHandle;
                    std::shared_ptr<PbdBody> body = pbdModel->getBody(bodyId);
                    fracturePullDistance = applyFragmentGrab(
                        fragmentGrabState,
                        getToolTipPosition(forcepsToolObj),
                        body,
                        latestSceneDt);
                    if (!fractureComplete && fracturePullDistance >= FORCEPS_TEAR_TRIGGER_DISTANCE)
                    {
                        applyFractureVisualGap(
                            meniscusTissue,
                            fracturePath,
                            fracturePath.length);
                        hideSceneObjectVisuals(meniscusTissue.object);
                        showSceneObjectVisuals(sutureMeniscusTissue.object);
                        showSceneObjectVisuals(fragmentTissue.object);
                        fragmentFollowStartToolTip = getToolTipPosition(forcepsToolObj);
                        fractureComplete = true;
                        std::cout << "PBDMeniscusHapticSuture: forceps tear complete after pull "
                                  << fracturePullDistance
                                  << "; hidden tear-interface faces and switched to separated fragment/main body. Press N for suture stage."
                                  << std::endl;
                    }
                }

                if (fractureComplete && fragmentGrabState.active && fragmentTissue.object != nullptr)
                {
                    const Vec3d currentTip = getToolTipPosition(forcepsToolObj);
                    const Vec3d delta = currentTip - fragmentFollowStartToolTip;
                    translateTissueByDelta(fragmentTissue, delta);
                    fragmentFollowStartToolTip = currentTip;
                }

                if (!fractureComplete)
                {
                    meniscusTissue.tetMesh->getVertexPositions()->postModified();
                    meniscusTissue.tetMesh->postModified();
                    meniscusTissue.surfaceMesh->getVertexPositions()->postModified();
                    meniscusTissue.surfaceMesh->computeVertexNormals();
                    meniscusTissue.surfaceMesh->postModified();
                }
                return;
            }

            if (leftToolObj == nullptr || needleObj == nullptr || sutureThreadObj == nullptr)
            {
                return;
            }

            syncNeedleRigidFollow(leftToolObj);

            const bool punctureDetectionActive =
                ENABLE_AUTO_SUTURE_ANCHORS && leftGraspActive && needleRigidFollowActive;
            if (punctureDetectionActive)
            {
                if (anchorCooldownFrames > 0)
                {
                    anchorCooldownFrames--;
                }

                auto needleLineMesh = std::dynamic_pointer_cast<LineMesh>(needleObj->getPhysicsGeometry());
                if (needleLineMesh != nullptr && needleLineMesh->getNumVertices() > 1)
                {
                    const VecDataArray<double, 3>& needleVertices = *needleLineMesh->getVertexPositions();
                    const Vec3d& needleTip = needleVertices[needleVertices.size() - 1];
                    const SurfaceHit hit = findClosestSurfaceHit(meniscusTissue.surfaceMesh, needleTip);
                    if (hit.valid)
                    {
                        const bool crossedSurface =
                            havePreviousNeedleTipHit
                            && previousNeedleTipSignedDistance * hit.signedDistance < 0.0
                            && std::abs(previousNeedleTipSignedDistance) > PUNCTURE_SIGN_EPSILON
                            && std::abs(hit.signedDistance) > PUNCTURE_SIGN_EPSILON
                            && hit.distance < PUNCTURE_SURFACE_DISTANCE;
                        if (crossedSurface && anchorCooldownFrames == 0)
                        {
                            addSutureAnchor(hit);
                        }

                        previousNeedleTipSignedDistance = hit.signedDistance;
                        havePreviousNeedleTipHit = true;
                    }
                }
            }

            if (leftGraspActive)
            {
                if (leftThreadGrasping == nullptr)
                {
                    return;
                }
                if (leftManualNeedleConstraints.empty())
                {
                    const bool beganNeedleGrasp = tryBeginManualNeedleGrasp(
                        "left haptic",
                        leftToolObj,
                        leftManualNeedleConstraints,
                        leftManualNeedleConstraintPtrs);
                    if (beganNeedleGrasp)
                    {
                        beginNeedleRigidFollow(leftToolObj);
                    }
                }
                else if (!needleRigidFollowActive)
                {
                    beginNeedleRigidFollow(leftToolObj);
                }
                leftThreadGrasping->regrasp();
            }

            if (!leftManualNeedleConstraintPtrs.empty())
            {
                pbdModel->getSolver()->addConstraints(&leftManualNeedleConstraintPtrs);
            }

            if (ENABLE_MENISCUS_DEFORMATION)
            {
                meniscusTissue.tetMesh->getVertexPositions()->postModified();
                meniscusTissue.tetMesh->postModified();
                meniscusTissue.surfaceMesh->getVertexPositions()->postModified();
                meniscusTissue.surfaceMesh->computeVertexNormals();
                meniscusTissue.surfaceMesh->postModified();
                if (SHOW_MENISCUS_EDGE_OVERLAY && meniscusTissue.edgeMesh != nullptr)
                {
                    meniscusTissue.edgeMesh->getVertexPositions()->postModified();
                    meniscusTissue.edgeMesh->postModified();
                }
            }
        });

    queueConnect<KeyEvent>(viewer->getKeyboardDevice(), &KeyboardDeviceClient::keyPress, sceneManager,
        [&](KeyEvent* e)
        {
            if (e->m_key == 'n' || e->m_key == 'N')
            {
                if (!fractureComplete)
                {
                    std::cout << "PBDMeniscusHapticSuture: finish tear before suturing." << std::endl;
                    return;
                }
                activateSutureStage();
#ifdef iMSTK_USE_HAPTICS
                if (leftToolObj != nullptr)
                {
                    auto leftController = leftToolObj->getComponent<PbdObjectController>();
                    leftController->setDevice(leftDeviceClient);
                    leftController->setTranslationScaling(HAPTIC_TRANSLATION_SCALING);
                    leftController->setTranslationOffset(hapticWorkspaceOffset);
                    leftController->setUseSpring(ENABLE_HAPTIC_FORCE_FEEDBACK);
                    leftController->setForceScaling(ENABLE_HAPTIC_FORCE_FEEDBACK ? 0.01 : 0.0);
                }
#endif
                return;
            }
            if (demoStage != DemoStage::Suture || leftToolObj == nullptr || leftThreadGrasping == nullptr)
            {
                return;
            }
            if (e->m_key == 'g')
            {
                auto graspCapsule = std::dynamic_pointer_cast<Capsule>(
                    leftToolObj->getVisualModel(1)->getGeometry());
                const bool beganNeedleGrasp = tryBeginManualNeedleGrasp(
                    "left keyboard",
                    leftToolObj,
                    leftManualNeedleConstraints,
                    leftManualNeedleConstraintPtrs);
                if (beganNeedleGrasp)
                {
                    beginNeedleRigidFollow(leftToolObj);
                }
                leftThreadGrasping->beginCellGrasp(graspCapsule);
                leftGraspActive = true;
                return;
            }
            if (e->m_key == 'f')
            {
                clearManualNeedleGrasp(leftManualNeedleConstraints, leftManualNeedleConstraintPtrs);
                endNeedleRigidFollow();
                leftThreadGrasping->endGrasp();
                leftGraspActive = false;
            }
        });

    std::cout << "PBDMeniscusHapticSuture: imported tetrahedral VTK mesh is ready." << std::endl;
    std::cout << "PBDMeniscusHapticSuture: single haptic device starts on the forceps tear tool." << std::endl;
    std::cout << "PBDMeniscusHapticSuture: haptic force feedback is "
              << (ENABLE_HAPTIC_FORCE_FEEDBACK ? "enabled." : "disabled for initial testing.") << std::endl;
    std::cout << "PBDMeniscusHapticSuture: hold the TouchX haptic button near the red route and pull to tear; press N after tear completion for suturing." << std::endl;
    std::cout << "PBDMeniscusHapticSuture: mouse-controlled tool is disabled; mouse input only affects the viewer controls." << std::endl;

    driver->start();
    return 0;
}
