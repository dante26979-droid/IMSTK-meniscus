/*
** This file is part of the Interactive Medical Simulation Toolkit (iMSTK)
** iMSTK is distributed under the Apache License, Version 2.0.
** See accompanying NOTICE for details.
*/

#include "imstkCamera.h"
#include "imstkDirectionalLight.h"
#include "imstkDummyClient.h"
#include "imstkIsometricMap.h"
#include "imstkKeyboardDeviceClient.h"
#include "imstkKeyboardSceneControl.h"
#include "imstkLineMesh.h"
#include "imstkMeshIO.h"
#include "imstkMouseDeviceClient.h"
#include "imstkMouseSceneControl.h"
#include "imstkNew.h"
#include "imstkPbdConstraintContainer.h"
#include "imstkPbdDistanceConstraint.h"
#include "imstkPbdModel.h"
#include "imstkPbdModelConfig.h"
#include "imstkPbdObject.h"
#include "imstkPbdObjectStitching.h"
#include "imstkRbdConstraint.h"
#include "imstkRenderMaterial.h"
#include "imstkRigidBodyModel2.h"
#include "imstkRigidObject2.h"
#include "imstkRigidObjectController.h"
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
#include <map>
#include <set>
#include <sstream>
#include <vector>

using namespace imstk;

using Edge = std::array<int, 2>;
using Face = std::array<int, 3>;

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

static std::shared_ptr<SurfaceMesh>
makeBoundarySurfaceMesh(const std::shared_ptr<TetrahedralMesh> tetMesh)
{
    auto surfaceMesh = std::make_shared<SurfaceMesh>();
    surfaceMesh->initialize(tetMesh->getVertexPositions(), buildBoundarySurfaceCells(tetMesh));
    return surfaceMesh;
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
    for (const Edge& edge : tetEdges)
    {
        auto constraint = std::make_shared<PbdDistanceConstraint>();
        constraint->initConstraint(
            tissue.tetMesh->getVertexPosition(edge[0]),
            tissue.tetMesh->getVertexPosition(edge[1]),
            PbdParticleId(bodyId, edge[0]),
            PbdParticleId(bodyId, edge[1]),
            1.0e4);
        pbdModel->getConstraints()->addConstraint(constraint);
    }
    pbdModel->getConfig()->setBodyDamping(bodyId, 0.055, 0.0);

    std::shared_ptr<RenderMaterial> material = tissueObj->getVisualModel(0)->getRenderMaterial();
    if (material != nullptr)
    {
        material->setDisplayMode(RenderMaterial::DisplayMode::Surface);
        material->setOpacity(1.0);
        material->setBackFaceCulling(false);
        material->setIsDynamicMesh(true);
        material->setShadingModel(RenderMaterial::ShadingModel::Phong);
        material->setDiffuseColor(Color(0.76, 0.76, 0.72));
    }

    std::cout << "PBDMeniscusHapticSuture: loaded "
              << tissue.tetMesh->getNumVertices() << " vertices, "
              << tissue.tetMesh->getNumCells() << " tetrahedra, "
              << tetEdges.size() << " distance constraints from "
              << IMPORTED_MENISCUS_VTK_PATH << std::endl;
    std::cout << "PBDMeniscusHapticSuture: fixed "
              << tissueObj->getPbdBody()->fixedNodeIds.size()
              << " support vertices on horns/posterior rim." << std::endl;
    return tissue;
}

static std::shared_ptr<RigidObject2>
makeSutureToolObj()
{
    auto toolGeom = std::make_shared<LineMesh>();
    auto vertices = std::make_shared<VecDataArray<double, 3>>(2);
    (*vertices)[0] = Vec3d(0.0, 0.0, 0.0);
    (*vertices)[1] = Vec3d(0.0, 2.5, 0.0);
    auto indices = std::make_shared<VecDataArray<int, 2>>(1);
    (*indices)[0] = Vec2i(0, 1);
    toolGeom->initialize(vertices, indices);

    auto toolObj = std::make_shared<RigidObject2>("Haptic suture ray tool");
    toolObj->setCollidingGeometry(toolGeom);
    toolObj->setPhysicsGeometry(toolGeom);

    std::shared_ptr<SurfaceMesh> forcepsMesh = MeshIO::read<SurfaceMesh>(FORCEPS_TOOL_PATH);
    if (forcepsMesh != nullptr)
    {
        Vec3d boundsMin = Vec3d::Zero();
        Vec3d boundsMax = Vec3d::Zero();
        forcepsMesh->computeBoundingBox(boundsMin, boundsMax);
        const Vec3d extent = boundsMax - boundsMin;
        const double maxExtent = std::max(extent[0], std::max(extent[1], extent[2]));
        const double scale = (maxExtent > 1.0e-8) ? 2.5 / maxExtent : 1.0;

        auto computeBandCenter = [&](const double yMin, const double yMax) -> Vec3d
            {
                Vec3d center = Vec3d::Zero();
                int count = 0;
                const VecDataArray<double, 3>& positions = *forcepsMesh->getVertexPositions();
                for (int i = 0; i < positions.size(); i++)
                {
                    const Vec3d& p = positions[i];
                    if (p[1] < yMin || p[1] > yMax)
                    {
                        continue;
                    }
                    center += p;
                    count++;
                }
                if (count == 0)
                {
                    return Vec3d(
                        (boundsMin[0] + boundsMax[0]) * 0.5,
                        (yMin + yMax) * 0.5,
                        (boundsMin[2] + boundsMax[2]) * 0.5);
                }
                return center / static_cast<double>(count);
            };

        const double yExtent = std::max(1.0e-8, extent[1]);
        const Vec3d axisBase = computeBandCenter(boundsMin[1], boundsMin[1] + yExtent * 0.06);
        const Vec3d axisHead = computeBandCenter(
            boundsMin[1] + yExtent * 0.38,
            boundsMin[1] + yExtent * 0.50);
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

        std::shared_ptr<RenderMaterial> forcepsMaterial =
            toolObj->getVisualModel(0)->getRenderMaterial();
        forcepsMaterial->setDisplayMode(RenderMaterial::DisplayMode::Surface);
        forcepsMaterial->setDiffuseColor(Color(0.74, 0.76, 0.78));
        forcepsMaterial->setShadingModel(RenderMaterial::ShadingModel::PBR);
        forcepsMaterial->setRoughness(0.28);
        forcepsMaterial->setMetalness(0.75);
        forcepsMaterial->setIsDynamicMesh(true);
        forcepsMaterial->setBackFaceCulling(false);
    }
    else
    {
        toolObj->setVisualGeometry(toolGeom);
    }

    auto rayMaterial = std::make_shared<RenderMaterial>();
    rayMaterial->setDisplayMode(RenderMaterial::DisplayMode::Wireframe);
    rayMaterial->setColor(Color(0.05, 1.0, 0.25));
    rayMaterial->setLineWidth(5.0);
    rayMaterial->setShadingModel(RenderMaterial::ShadingModel::PBR);
    rayMaterial->setRoughness(0.35);
    rayMaterial->setMetalness(0.0);
    rayMaterial->setIsDynamicMesh(false);

    imstkNew<VisualModel> rayVisual;
    rayVisual->setGeometry(toolGeom);
    rayVisual->setRenderMaterial(rayMaterial);
    toolObj->addVisualModel(rayVisual);

    auto rbdModel = std::make_shared<RigidBodyModel2>();
    rbdModel->getConfig()->m_gravity = Vec3d::Zero();
    rbdModel->getConfig()->m_maxNumIterations = 5;
    toolObj->setDynamicalModel(rbdModel);

    toolObj->getRigidBody()->m_mass = 0.3;
    toolObj->getRigidBody()->m_intertiaTensor = Mat3d::Identity() * 10000.0;
    toolObj->getRigidBody()->m_initPos = Vec3d(1.1, 1.2, 0.0);
    toolObj->getRigidBody()->m_initOrientation = Quatd(Rotd(0.0, Vec3d::UnitZ()));

    auto controller = toolObj->addComponent<RigidObjectController>();
    controller->setControlledObject(toolObj);
    controller->setLinearKs(1000.0);
    controller->setAngularKs(10000000.0);
    controller->setUseCritDamping(true);
    controller->setForceScaling(0.0045);
    controller->setSmoothingKernelSize(15);
    controller->setUseForceSmoothening(true);
    controller->setTranslationScaling(1.0);
    controller->setTranslationOffset(Vec3d::Zero());

    return toolObj;
}

static Vec3d
getToolTipPosition(const std::shared_ptr<RigidObject2> toolObj)
{
    auto toolGeom = std::dynamic_pointer_cast<LineMesh>(toolObj->getCollidingGeometry());
    if (toolGeom == nullptr || toolGeom->getNumVertices() == 0)
    {
        return toolObj->getRigidBody()->m_initPos;
    }
    return toolGeom->getVertexPosition(0);
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
    pbdModel->getConfig()->m_dt = 0.001;
    pbdModel->getConfig()->m_iterations = 4;
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
    if (meniscusTissue.edgeMesh != nullptr)
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

    auto stitching = std::make_shared<PbdObjectStitching>(meniscusTissue.object);
    stitching->setStiffness(0.65);
    stitching->setStitchDistance(2.5);
    scene->addInteraction(stitching);

    std::shared_ptr<RigidObject2> sutureToolObj = makeSutureToolObj();
    scene->addSceneObject(sutureToolObj);

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
    driver->setDesiredDt(1.0 / 240.0);

#ifdef iMSTK_USE_HAPTICS
    std::shared_ptr<DeviceManager> hapticManager = DeviceManagerFactory::makeDeviceManager();
    std::shared_ptr<DeviceClient> hapticDeviceClient = hapticManager->makeDeviceClient();
    auto deviceClient = std::make_shared<DummyClient>();
    driver->addModule(hapticManager);

    const Vec3d workspaceCenter =
        (meniscusTissue.boundsMin + meniscusTissue.boundsMax) * 0.5 + Vec3d(0.0, 1.35, 1.55);
    connect<Event>(sceneManager, &SceneManager::preUpdate,
        [&](Event*)
        {
            const Vec3d rawPos = hapticDeviceClient->getPosition();
            constexpr double kHapticWorkspaceScale = 8.5;
            const Vec3d mappedPos(
                rawPos[0] * kHapticWorkspaceScale + workspaceCenter[0],
                rawPos[1] * kHapticWorkspaceScale + workspaceCenter[1],
                rawPos[2] * kHapticWorkspaceScale + workspaceCenter[2]);
            deviceClient->setPosition(mappedPos);

            const Quatd rawOrientation = hapticDeviceClient->getOrientation();
            const Quatd deviceToScene(Rotd(-PI_2, Vec3d::UnitX()));
            const Quatd toolAxisToDeviceShaft =
                Quatd::FromTwoVectors(Vec3d::UnitY(), Vec3d::UnitZ());
            deviceClient->setOrientation(deviceToScene * rawOrientation * toolAxisToDeviceShaft);
        });
#else
    auto deviceClient = std::make_shared<DummyClient>();
    connect<Event>(sceneManager, &SceneManager::postUpdate,
        [&](Event*)
        {
            const Vec2d& mousePos = viewer->getMouseDevice()->getPos();
            deviceClient->setPosition(Vec3d(mousePos[0] * 4.0, 0.0, mousePos[1] * 3.0));
            deviceClient->setOrientation(Quatd(Rotd(0.0, Vec3d::UnitZ())));
        });
#endif

    auto sutureToolController = sutureToolObj->getComponent<RigidObjectController>();
    sutureToolController->setDevice(deviceClient);

    int stitchCount = 0;
    double latestSceneDt = driver->getDesiredDt();

    auto performToolStitch = [&]()
        {
            auto toolGeom = std::dynamic_pointer_cast<LineMesh>(sutureToolObj->getCollidingGeometry());
            if (toolGeom == nullptr)
            {
                return;
            }

            const Vec3d& v1 = toolGeom->getVertexPosition(0);
            const Vec3d& v2 = toolGeom->getVertexPosition(1);
            const Vec3d direction = v1 - v2;
            if (direction.norm() < 1.0e-8)
            {
                return;
            }

            stitching->beginStitch(v1, direction.normalized(), 4.0);
            stitchCount++;
            std::cout << "PBDMeniscusHapticSuture: stitch " << stitchCount
                      << " from [" << v1.transpose() << "] along ["
                      << direction.normalized().transpose() << "]" << std::endl;
        };

#ifdef iMSTK_USE_HAPTICS
    queueConnect<ButtonEvent>(hapticDeviceClient, &DeviceClient::buttonStateChanged, sceneManager,
        [&](ButtonEvent* e)
        {
            if (e->m_button == 0 && e->m_buttonState == BUTTON_PRESSED)
            {
                performToolStitch();
            }
        });
#endif

    scene->addSceneObject(SimulationUtils::createDefaultSceneControl(driver));

    imstkNew<SceneObject> diagnosticsObj("PBDMeniscusHapticSuture diagnostics");
    auto diagnosticsText = diagnosticsObj->addComponent<TextVisualModel>("PBDMeniscusHapticSutureDiagnosticsText");
    diagnosticsText->setPosition(TextVisualModel::DisplayPosition::LowerLeft);
    diagnosticsText->setFontSize(24.0);
    diagnosticsText->setTextColor(Color::White);
    diagnosticsText->setText("FPS: -- | sim dt: -- ms | stitches: --");

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
                   << " | stitches: " << stitchCount;
            diagnosticsText->setText(stream.str());

            visualDtAccum = 0.0;
            visualFrameCount = 0;
        });
    scene->addSceneObject(diagnosticsObj);

    connect<Event>(sceneManager, &SceneManager::preUpdate,
        [&](Event*)
        {
            latestSceneDt = sceneManager->getDt();
            pbdModel->getConfig()->m_dt = latestSceneDt;
        });

    connect<Event>(sceneManager, &SceneManager::postUpdate,
        [&](Event*)
        {
            meniscusTissue.tetMesh->getVertexPositions()->postModified();
            meniscusTissue.tetMesh->postModified();
            meniscusTissue.surfaceMesh->getVertexPositions()->postModified();
            meniscusTissue.surfaceMesh->postModified();
            if (meniscusTissue.edgeMesh != nullptr)
            {
                meniscusTissue.edgeMesh->getVertexPositions()->postModified();
                meniscusTissue.edgeMesh->postModified();
            }
        });

    queueConnect<KeyEvent>(viewer->getKeyboardDevice(), &KeyboardDeviceClient::keyPress, sceneManager,
        [&](KeyEvent* e)
        {
#ifndef iMSTK_USE_HAPTICS
            if (e->m_key == 's')
            {
                performToolStitch();
                return;
            }
#endif
            if (e->m_key == 'r')
            {
                stitching->removeStitchConstraints();
                stitchCount = 0;
                std::cout << "PBDMeniscusHapticSuture: cleared stitch constraints." << std::endl;
            }
        });

    std::cout << "PBDMeniscusHapticSuture: imported tetrahedral VTK mesh is ready for haptic stitching." << std::endl;
    std::cout << "PBDMeniscusHapticSuture: press the haptic button to place a stitch with the green ray." << std::endl;
#ifndef iMSTK_USE_HAPTICS
    std::cout << "PBDMeniscusHapticSuture: haptics disabled; press 's' to place a mouse-driven test stitch." << std::endl;
#endif
    std::cout << "PBDMeniscusHapticSuture: press 'r' to clear stitch constraints." << std::endl;

    driver->start();
    return 0;
}
