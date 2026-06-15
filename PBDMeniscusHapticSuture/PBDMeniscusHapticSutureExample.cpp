/*
** This file is part of the Interactive Medical Simulation Toolkit (iMSTK)
** iMSTK is distributed under the Apache License, Version 2.0.
** See accompanying NOTICE for details.
*/

#include "imstkCamera.h"
#include "imstkCapsule.h"
#include "imstkDirectionalLight.h"
#include "imstkDummyClient.h"
#include "imstkGeometryUtilities.h"
#include "imstkIsometricMap.h"
#include "imstkKeyboardDeviceClient.h"
#include "imstkKeyboardSceneControl.h"
#include "imstkLineMesh.h"
#include "imstkMeshIO.h"
#include "imstkMouseDeviceClient.h"
#include "imstkMouseSceneControl.h"
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
#include "imstkPlane.h"
#include "imstkRenderMaterial.h"
#include "imstkScene.h"
#include "imstkSceneManager.h"
#include "imstkSimulationManager.h"
#include "imstkSimulationUtils.h"
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
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
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
static constexpr double MENISCUS_EDGE_STIFFNESS = 5.0e4;
static constexpr double LAP_TOOL_SCALE = 4.0;
static constexpr double NEEDLE_SCALE = 25.0;
static constexpr double GRASP_CAPSULE_RADIUS = 0.05;
static constexpr double GRASP_CAPSULE_LENGTH = 0.18;
static constexpr double MANUAL_NEEDLE_GRASP_TOLERANCE = 0.08;
static constexpr bool ENABLE_AUTO_SUTURE_ANCHORS = true;
static constexpr int MAX_SUTURE_ANCHORS = 8;
static constexpr double PUNCTURE_SURFACE_DISTANCE = 0.18;
static constexpr double PUNCTURE_SIGN_EPSILON = 0.005;

struct MeniscusTissue
{
    std::shared_ptr<PbdObject> object;
    std::shared_ptr<TetrahedralMesh> tetMesh;
    std::shared_ptr<SurfaceMesh> surfaceMesh;
    std::shared_ptr<LineMesh> edgeMesh;
    Vec3d boundsMin = Vec3d::Zero();
    Vec3d boundsMax = Vec3d::Zero();
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
makeImportedMeniscusObject(const std::shared_ptr<PbdModel> pbdModel)
{
    MeniscusTissue tissue;
    tissue.tetMesh = MeshIO::read<TetrahedralMesh>(IMPORTED_MENISCUS_VTK_PATH);
    if (tissue.tetMesh == nullptr)
    {
        std::cout << "PBDMeniscusHapticSuture: failed to load tetrahedral mesh "
                  << IMPORTED_MENISCUS_VTK_PATH << std::endl;
        return tissue;
    }

    centerAndScaleTetMesh(tissue.tetMesh);
    tissue.tetMesh->computeBoundingBox(tissue.boundsMin, tissue.boundsMax);
    tissue.surfaceMesh = makeBoundarySurfaceMesh(tissue.tetMesh);
    tissue.edgeMesh = makeBoundaryEdgeMesh(tissue.tetMesh, tissue.surfaceMesh);

    imstkNew<PbdObject> tissueObj("Imported left meniscus PBD tissue");
    tissue.object = tissueObj;
    tissueObj->setPhysicsGeometry(tissue.tetMesh);
    tissueObj->setVisualGeometry(tissue.surfaceMesh);
    tissueObj->setCollidingGeometry(tissue.surfaceMesh);
    tissueObj->setDynamicalModel(pbdModel);
    tissueObj->getPbdBody()->uniformMassValue = 0.06;
    tissueObj->getPbdBody()->fixedNodeIds =
        selectSupportNodes(tissue.tetMesh, tissue.boundsMin, tissue.boundsMax);

    const int bodyId = tissueObj->getPbdBody()->bodyHandle;
    const std::vector<Edge> tetEdges = getUniqueTetEdges(tissue.tetMesh);
    if (ENABLE_MENISCUS_DEFORMATION)
    {
        for (const Edge& edge : tetEdges)
        {
            auto constraint = std::make_shared<PbdDistanceConstraint>();
            constraint->initConstraint(
                tissue.tetMesh->getVertexPosition(edge[0]),
                tissue.tetMesh->getVertexPosition(edge[1]),
                PbdParticleId(bodyId, edge[0]),
                PbdParticleId(bodyId, edge[1]),
                MENISCUS_EDGE_STIFFNESS);
            pbdModel->getConstraints()->addConstraint(constraint);
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
              << (ENABLE_MENISCUS_DEFORMATION ? tetEdges.size() : 0) << " distance constraints from "
              << IMPORTED_MENISCUS_VTK_PATH << std::endl;
    std::cout << "PBDMeniscusHapticSuture: fixed "
              << tissueObj->getPbdBody()->fixedNodeIds.size()
              << " support vertices on horns/posterior rim." << std::endl;
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

    const Vec3d tissueCenter = (meniscusTissue.boundsMin + meniscusTissue.boundsMax) * 0.5;
    const Vec3d hapticWorkspaceOffset = tissueCenter + Vec3d(0.0, 1.9, 1.0);
    const Vec3d leftToolStart = hapticWorkspaceOffset + Vec3d(0.0, 0.0, 0.15 * LAP_TOOL_SCALE);
    const Vec3d rightToolStart = tissueCenter + Vec3d(1.2, 1.25, 1.2);
    std::shared_ptr<PbdObject> leftToolObj =
        makeLapToolObj("leftHapticLapTool", pbdModel, leftToolStart);
    scene->addSceneObject(leftToolObj);
    std::shared_ptr<PbdObject> rightToolObj =
        makeLapToolObj("rightMouseLapTool", pbdModel, rightToolStart);
    scene->addSceneObject(rightToolObj);

    std::shared_ptr<PbdObject> needleObj =
        makeNeedleObj(pbdModel, tissueCenter + Vec3d(0.25, 0.8, 0.8));
    scene->addSceneObject(needleObj);

    std::shared_ptr<PbdObject> sutureThreadObj = makePbdString(
        "sutureThread",
        tissueCenter + Vec3d(0.25, 0.8, 0.8),
        Vec3d(0.0, 0.0, 1.0),
        50,
        2.2,
        needleObj);
    scene->addSceneObject(sutureThreadObj);

    auto toolCollision = std::make_shared<PbdObjectCollision>(leftToolObj, rightToolObj);
    toolCollision->setRigidBodyCompliance(0.00001);
    scene->addInteraction(toolCollision);

    if (ENABLE_TOOL_NEEDLE_CONTACT)
    {
        auto leftNeedleCollision = std::make_shared<PbdObjectCollision>(leftToolObj, needleObj);
        leftNeedleCollision->setRigidBodyCompliance(0.0001);
        leftNeedleCollision->setUseCorrectVelocity(false);
        scene->addInteraction(leftNeedleCollision);
        auto rightNeedleCollision = std::make_shared<PbdObjectCollision>(rightToolObj, needleObj);
        rightNeedleCollision->setRigidBodyCompliance(0.0001);
        rightNeedleCollision->setUseCorrectVelocity(false);
        scene->addInteraction(rightNeedleCollision);
    }

    if (ENABLE_TOOL_THREAD_CONTACT)
    {
        auto leftThreadCollision = std::make_shared<PbdObjectCollision>(leftToolObj, sutureThreadObj);
        leftThreadCollision->setRigidBodyCompliance(0.0001);
        leftThreadCollision->setUseCorrectVelocity(false);
        scene->addInteraction(leftThreadCollision);
        auto rightThreadCollision = std::make_shared<PbdObjectCollision>(rightToolObj, sutureThreadObj);
        rightThreadCollision->setRigidBodyCompliance(0.0001);
        rightThreadCollision->setUseCorrectVelocity(false);
        scene->addInteraction(rightThreadCollision);
    }

    if (ENABLE_NEEDLE_TISSUE_CONTACT)
    {
        auto needleTissueCollision = std::make_shared<PbdObjectCollision>(needleObj, meniscusTissue.object);
        needleTissueCollision->setRigidBodyCompliance(0.00005);
        needleTissueCollision->setUseCorrectVelocity(false);
        scene->addInteraction(needleTissueCollision);
    }
    if (ENABLE_THREAD_TISSUE_CONTACT)
    {
        auto threadTissueCollision = std::make_shared<PbdObjectCollision>(sutureThreadObj, meniscusTissue.object);
        threadTissueCollision->setDeformableStiffnessA(0.05);
        threadTissueCollision->setDeformableStiffnessB(0.05);
        threadTissueCollision->setUseCorrectVelocity(false);
        scene->addInteraction(threadTissueCollision);
    }

    auto leftNeedleGrasping = std::make_shared<PbdObjectGrasping>(needleObj, leftToolObj);
    leftNeedleGrasping->setCompliance(0.00001);
    scene->addInteraction(leftNeedleGrasping);
    auto leftThreadGrasping = std::make_shared<PbdObjectGrasping>(sutureThreadObj, leftToolObj);
    leftThreadGrasping->setCompliance(0.00001);
    scene->addInteraction(leftThreadGrasping);
    auto rightNeedleGrasping = std::make_shared<PbdObjectGrasping>(needleObj, rightToolObj);
    rightNeedleGrasping->setCompliance(0.00001);
    scene->addInteraction(rightNeedleGrasping);
    auto rightThreadGrasping = std::make_shared<PbdObjectGrasping>(sutureThreadObj, rightToolObj);
    rightThreadGrasping->setCompliance(0.00001);
    scene->addInteraction(rightThreadGrasping);

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
    std::vector<std::shared_ptr<PbdBodyToBodyDistanceConstraint>> rightManualNeedleConstraints;
    std::vector<PbdConstraint*> leftManualNeedleConstraintPtrs;
    std::vector<PbdConstraint*> rightManualNeedleConstraintPtrs;

    auto clearManualNeedleGrasp =
        [](std::vector<std::shared_ptr<PbdBodyToBodyDistanceConstraint>>& constraints,
           std::vector<PbdConstraint*>& constraintPtrs)
        {
            constraints.clear();
            constraintPtrs.clear();
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
            anchorCooldownFrames = 30;

            std::cout << "PBDMeniscusHapticSuture: added suture anchor "
                      << sutureAnchorConstraints.size()
                      << " at tissue triangle " << hit.triangleId
                      << " using thread vertex " << closestThreadVertex << std::endl;
        };

    if (ENABLE_THREAD_SELF_COLLISION)
    {
        auto threadSelfCollision = std::make_shared<PbdObjectCollision>(sutureThreadObj, sutureThreadObj);
        threadSelfCollision->setDeformableStiffnessA(0.05);
        threadSelfCollision->setDeformableStiffnessB(0.05);
        scene->addInteraction(threadSelfCollision);
    }

    auto mousePlane = std::make_shared<Plane>(
        tissueCenter + Vec3d(0.0, 1.2, 1.4),
        Vec3d(0.1, 0.0, 1.0));
    mousePlane->setWidth(6.0);

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
    bool rightGraspActive = false;
    double rightDummyOffset = -0.07 * LAP_TOOL_SCALE;

#ifdef iMSTK_USE_HAPTICS
    std::shared_ptr<DeviceManager> hapticManager = DeviceManagerFactory::makeDeviceManager();
    std::shared_ptr<DeviceClient> leftDeviceClient = hapticManager->makeDeviceClient("Default Device");
    driver->addModule(hapticManager);

    auto leftController = leftToolObj->getComponent<PbdObjectController>();
    leftController->setDevice(leftDeviceClient);
    leftController->setTranslationScaling(HAPTIC_TRANSLATION_SCALING);
    leftController->setTranslationOffset(hapticWorkspaceOffset);
    leftController->setUseSpring(ENABLE_HAPTIC_FORCE_FEEDBACK);
    leftController->setForceScaling(ENABLE_HAPTIC_FORCE_FEEDBACK ? 0.01 : 0.0);
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
            if (e->m_button != 0 && e->m_button != 1)
            {
                return;
            }

            if (e->m_buttonState == BUTTON_PRESSED)
            {
                auto graspCapsule = std::dynamic_pointer_cast<Capsule>(
                    leftToolObj->getVisualModel(1)->getGeometry());
                tryBeginManualNeedleGrasp(
                    "left haptic",
                    leftToolObj,
                    leftManualNeedleConstraints,
                    leftManualNeedleConstraintPtrs);
                leftThreadGrasping->beginCellGrasp(graspCapsule);
                leftGraspActive = true;
                std::cout << "PBDMeniscusHapticSuture: left haptic grasp button "
                          << e->m_button << " pressed." << std::endl;
            }
            else if (e->m_buttonState == BUTTON_RELEASED)
            {
                clearManualNeedleGrasp(leftManualNeedleConstraints, leftManualNeedleConstraintPtrs);
                leftThreadGrasping->endGrasp();
                leftGraspActive = false;
                std::cout << "PBDMeniscusHapticSuture: left haptic grasp button "
                          << e->m_button << " released." << std::endl;
            }
        });
#else
    std::cout << "PBDMeniscusHapticSuture: haptics are not compiled; the left tool is stationary." << std::endl;
#endif

    auto rightDeviceClient = std::make_shared<DummyClient>();
    auto rightController = rightToolObj->getComponent<PbdObjectController>();
    rightController->setDevice(rightDeviceClient);
    rightController->setUseSpring(false);

    double latestSceneDt = driver->getDesiredDt();

    scene->addSceneObject(SimulationUtils::createDefaultSceneControl(driver));

    imstkNew<SceneObject> diagnosticsObj("PBDMeniscusHapticSuture diagnostics");
    auto diagnosticsText = diagnosticsObj->addComponent<TextVisualModel>("PBDMeniscusHapticSutureDiagnosticsText");
    diagnosticsText->setPosition(TextVisualModel::DisplayPosition::LowerLeft);
    diagnosticsText->setFontSize(24.0);
    diagnosticsText->setTextColor(Color::White);
    diagnosticsText->setText("FPS: -- | sim dt: -- ms | anchors: -- | btn: -- | LN/LT/RN/RT: --");

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
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(1)
                   << "FPS: " << visualFps
                   << " | sim dt: " << latestSceneDt * 1000.0 << " ms"
                   << " | anchors: " << sutureAnchorConstraints.size()
                   << " | btn: " << (leftGraspActive ? "L" : "-") << (rightGraspActive ? "R" : "-")
                   << " | LN/LT/RN/RT: "
                   << (!leftManualNeedleConstraints.empty() ? "1" : "0")
                   << (leftThreadGrasping->hasConstraints() ? "1" : "0")
                   << (!rightManualNeedleConstraints.empty() ? "1" : "0")
                   << (rightThreadGrasping->hasConstraints() ? "1" : "0");
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
            std::shared_ptr<MouseDeviceClient> mouseDeviceClient = viewer->getMouseDevice();
            const Vec2d& mousePos = mouseDeviceClient->getPos();
            auto geom = std::dynamic_pointer_cast<Capsule>(rightToolObj->getPhysicsGeometry());
            Vec3d a = Vec3d::UnitY();
            Vec3d b = a.cross(mousePlane->getNormal()).normalized();
            a = b.cross(mousePlane->getNormal());
            const double width = mousePlane->getWidth();
            const Vec3d toolAxis =
                (geom != nullptr) ? geom->getOrientation().toRotationMatrix().col(1).normalized() : Vec3d::UnitY();
            rightDeviceClient->setPosition(
                mousePlane->getPosition()
                + a * width * (mousePos[1] - 0.5)
                + b * width * (mousePos[0] - 0.5)
                + toolAxis * rightDummyOffset);

            if (ENABLE_AUTO_SUTURE_ANCHORS)
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
                if (leftManualNeedleConstraints.empty())
                {
                    tryBeginManualNeedleGrasp(
                        "left haptic",
                        leftToolObj,
                        leftManualNeedleConstraints,
                        leftManualNeedleConstraintPtrs);
                }
                leftThreadGrasping->regrasp();
            }
            if (rightGraspActive)
            {
                if (rightManualNeedleConstraints.empty())
                {
                    tryBeginManualNeedleGrasp(
                        "right mouse",
                        rightToolObj,
                        rightManualNeedleConstraints,
                        rightManualNeedleConstraintPtrs);
                }
                rightThreadGrasping->regrasp();
            }

            if (!leftManualNeedleConstraintPtrs.empty())
            {
                pbdModel->getSolver()->addConstraints(&leftManualNeedleConstraintPtrs);
            }
            if (!rightManualNeedleConstraintPtrs.empty())
            {
                pbdModel->getSolver()->addConstraints(&rightManualNeedleConstraintPtrs);
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
            if (e->m_key == 'g')
            {
                auto graspCapsule = std::dynamic_pointer_cast<Capsule>(
                    leftToolObj->getVisualModel(1)->getGeometry());
                tryBeginManualNeedleGrasp(
                    "left keyboard",
                    leftToolObj,
                    leftManualNeedleConstraints,
                    leftManualNeedleConstraintPtrs);
                leftThreadGrasping->beginCellGrasp(graspCapsule);
                leftGraspActive = true;
                return;
            }
            if (e->m_key == 'f')
            {
                clearManualNeedleGrasp(leftManualNeedleConstraints, leftManualNeedleConstraintPtrs);
                leftThreadGrasping->endGrasp();
                leftGraspActive = false;
            }
        });

    connect<MouseEvent>(viewer->getMouseDevice(), &MouseDeviceClient::mouseScroll,
        [&](MouseEvent* e)
        {
            rightDummyOffset += e->m_scrollDx * 0.01;
        });
    connect<MouseEvent>(viewer->getMouseDevice(), &MouseDeviceClient::mouseButtonPress,
        [&](MouseEvent*)
        {
            auto graspCapsule = std::dynamic_pointer_cast<Capsule>(
                rightToolObj->getVisualModel(1)->getGeometry());
            tryBeginManualNeedleGrasp(
                "right mouse",
                rightToolObj,
                rightManualNeedleConstraints,
                rightManualNeedleConstraintPtrs);
            rightThreadGrasping->beginCellGrasp(graspCapsule);
            rightGraspActive = true;
        });
    connect<MouseEvent>(viewer->getMouseDevice(), &MouseDeviceClient::mouseButtonRelease,
        [&](MouseEvent*)
        {
            clearManualNeedleGrasp(rightManualNeedleConstraints, rightManualNeedleConstraintPtrs);
            rightThreadGrasping->endGrasp();
            rightGraspActive = false;
        });

    std::cout << "PBDMeniscusHapticSuture: imported tetrahedral VTK mesh is ready." << std::endl;
    std::cout << "PBDMeniscusHapticSuture: single haptic device controls the left lap tool." << std::endl;
    std::cout << "PBDMeniscusHapticSuture: haptic force feedback is "
              << (ENABLE_HAPTIC_FORCE_FEEDBACK ? "enabled." : "disabled for initial testing.") << std::endl;
    std::cout << "PBDMeniscusHapticSuture: hold haptic button 0/1 to grasp the needle or thread; release to let go." << std::endl;
    std::cout << "PBDMeniscusHapticSuture: the right lap tool is mouse controlled; mouse press grasps, release lets go, scroll changes depth." << std::endl;

    driver->start();
    return 0;
}
