/*
** This file is part of the Interactive Medical Simulation Toolkit (iMSTK)
** iMSTK is distributed under the Apache License, Version 2.0.
** See accompanying NOTICE for details.
*/

#include "PBDFractureTear.h"

#include "gtest/gtest.h"

using namespace imstk;

TEST(PbdFractureTearHelperTest, TearPathComputesLengthAndSamples)
{
    TearPath path({ Vec3d(0.0, 0.0, 0.0), Vec3d(2.0, 0.0, 0.0), Vec3d(2.0, 0.0, 2.0) });

    EXPECT_DOUBLE_EQ(path.getLength(), 4.0);
    EXPECT_EQ(path.sample(0.0), Vec3d(0.0, 0.0, 0.0));
    EXPECT_EQ(path.sample(1.0), Vec3d(1.0, 0.0, 0.0));
    EXPECT_EQ(path.sample(3.0), Vec3d(2.0, 0.0, 1.0));
    EXPECT_EQ(path.sample(99.0), Vec3d(2.0, 0.0, 2.0));
}

TEST(PbdFractureTearHelperTest, SurfaceBuilderGeneratesCurtainTriangles)
{
    TearPath path({ Vec3d(0.0, 0.0, 0.0), Vec3d(2.0, 0.0, 0.0), Vec3d(4.0, 0.0, 0.0) });
    PbdFractureSurfaceMeshBuilder builder;

    builder.update(path, 0.0, 0.5);
    EXPECT_EQ(builder.getSurfaceMesh()->getNumVertices(), 0);
    EXPECT_EQ(builder.getSurfaceMesh()->getNumTriangles(), 0);

    builder.update(path, 2.0, 0.5);
    EXPECT_EQ(builder.getSurfaceMesh()->getNumVertices(), 4);
    EXPECT_EQ(builder.getSurfaceMesh()->getNumTriangles(), 2);

    builder.update(path, 4.0, 0.5);
    EXPECT_EQ(builder.getSurfaceMesh()->getNumVertices(), 6);
    EXPECT_EQ(builder.getSurfaceMesh()->getNumTriangles(), 4);
}

TEST(PbdFractureTearHelperTest, SurfaceBuilderGeneratesSeparatedCurtainSides)
{
    TearPath path({ Vec3d(0.0, 0.0, 0.0), Vec3d(2.0, 0.0, 0.0) });
    PbdFractureSurfaceMeshBuilder builder;

    builder.updateSeparatedSides(path, 2.0, 0.5, Vec3d(0.0, 1.25, 0.0), 0.0);

    EXPECT_EQ(builder.getSurfaceMesh()->getNumVertices(), 8);
    EXPECT_EQ(builder.getSurfaceMesh()->getNumTriangles(), 4);

    const VecDataArray<double, 3>& vertices = *builder.getSurfaceMesh()->getVertexPositions();
    EXPECT_DOUBLE_EQ(vertices[0].y(), -0.5);
    EXPECT_DOUBLE_EQ(vertices[1].y(), 0.5);
    EXPECT_DOUBLE_EQ(vertices[2].y(), 0.75);
    EXPECT_DOUBLE_EQ(vertices[3].y(), 1.75);
}

TEST(PbdFractureTearHelperTest, SurfaceBuilderGeneratesSingleOffsetSide)
{
    TearPath path({ Vec3d(0.0, 0.0, 0.0), Vec3d(2.0, 0.0, 0.0) });
    PbdFractureSurfaceMeshBuilder builder;

    builder.updateSide(path, 2.0, 0.5, Vec3d(0.0, 1.25, 0.0), 0.0);

    EXPECT_EQ(builder.getSurfaceMesh()->getNumVertices(), 4);
    EXPECT_EQ(builder.getSurfaceMesh()->getNumTriangles(), 2);

    const VecDataArray<double, 3>& vertices = *builder.getSurfaceMesh()->getVertexPositions();
    EXPECT_DOUBLE_EQ(vertices[0].y(), 0.75);
    EXPECT_DOUBLE_EQ(vertices[1].y(), 1.75);
}

TEST(PbdFractureTearHelperTest, HorizontalLayerCutSurfaceUsesPathHeight)
{
    TearPath path({ Vec3d(-1.0, 0.25, 0.0), Vec3d(1.0, 0.25, 0.0) });
    TearCutSurface cutSurface(path, 1.0, TearCutSurface::Mode::HorizontalLayer);

    EXPECT_GT(cutSurface.signedDistance(Vec3d(0.0, 0.75, 0.0)), 0.0);
    EXPECT_LT(cutSurface.signedDistance(Vec3d(0.0, -0.25, 0.0)), 0.0);

    const TearCutSurfaceSample sample = cutSurface.closestSample(Vec3d(0.5, 3.0, 0.2));
    EXPECT_NEAR(sample.distanceAlongPath, 1.5, 1e-8);
    EXPECT_DOUBLE_EQ(sample.distanceToSurface, 0.0);
    EXPECT_EQ(sample.normal, Vec3d::UnitY());
}

TEST(PbdFractureTearHelperTest, TetCutSurfaceAnchorsToRequestedSide)
{
    auto vertices = std::make_shared<VecDataArray<double, 3>>();
    vertices->push_back(Vec3d(-1.0, 0.0, -0.2));
    vertices->push_back(Vec3d(1.0, 0.0, -0.2));
    vertices->push_back(Vec3d(1.0, 1.0, 0.2));
    vertices->push_back(Vec3d(1.0, 0.0, 0.8));

    auto cells = std::make_shared<VecDataArray<int, 4>>();
    cells->push_back(Vec4i(0, 1, 2, 3));

    auto tetMesh = std::make_shared<TetrahedralMesh>();
    tetMesh->initialize(vertices, cells);

    const std::vector<Vec3d> restPositions = { (*vertices)[0], (*vertices)[1], (*vertices)[2], (*vertices)[3] };
    TearPath path({ Vec3d(0.0, 0.0, -1.0), Vec3d(0.0, 0.0, 1.0) });
    TearCutSurface cutSurface(path, 2.0, TearCutSurface::Mode::VerticalCurtain);

    PbdFractureSurfaceMeshBuilder stationaryBuilder;
    PbdFractureSurfaceMeshBuilder liftedBuilder;

    (*vertices)[0] += Vec3d(0.0, 1.25, 0.0);

    liftedBuilder.updateTetCutSide(tetMesh, restPositions, *vertices, cutSurface, 2.0, 2.0, 1.0, 0.0);
    stationaryBuilder.updateTetCutSide(tetMesh, restPositions, *vertices, cutSurface, 2.0, 2.0, -1.0, 0.0);

    EXPECT_EQ(liftedBuilder.getSurfaceMesh()->getNumVertices(), 3);
    EXPECT_EQ(liftedBuilder.getSurfaceMesh()->getNumTriangles(), 1);
    EXPECT_EQ(stationaryBuilder.getSurfaceMesh()->getNumVertices(), 3);
    EXPECT_EQ(stationaryBuilder.getSurfaceMesh()->getNumTriangles(), 1);

    const VecDataArray<double, 3>& liftedVertices = *liftedBuilder.getSurfaceMesh()->getVertexPositions();
    const VecDataArray<double, 3>& stationaryVertices = *stationaryBuilder.getSurfaceMesh()->getVertexPositions();
    for (int i = 0; i < 3; i++)
    {
        EXPECT_GT(liftedVertices[i].y(), stationaryVertices[i].y() + 0.9);
    }
}

TEST(PbdFractureTearHelperTest, TetCutSurfaceCanRebuildHorizontalLayer)
{
    auto vertices = std::make_shared<VecDataArray<double, 3>>();
    vertices->push_back(Vec3d(0.0, -1.0, 0.0));
    vertices->push_back(Vec3d(1.0, 1.0, 0.0));
    vertices->push_back(Vec3d(0.0, 1.0, 1.0));
    vertices->push_back(Vec3d(-1.0, 1.0, 0.0));

    auto cells = std::make_shared<VecDataArray<int, 4>>();
    cells->push_back(Vec4i(0, 1, 2, 3));

    auto tetMesh = std::make_shared<TetrahedralMesh>();
    tetMesh->initialize(vertices, cells);

    const std::vector<Vec3d> restPositions = { (*vertices)[0], (*vertices)[1], (*vertices)[2], (*vertices)[3] };
    TearPath path({ Vec3d(-2.0, 0.0, 0.0), Vec3d(2.0, 0.0, 0.0) });
    TearCutSurface cutSurface(path, 2.0, TearCutSurface::Mode::HorizontalLayer);

    (*vertices)[1] += Vec3d(0.0, 1.0, 0.0);
    (*vertices)[2] += Vec3d(0.0, 1.0, 0.0);
    (*vertices)[3] += Vec3d(0.0, 1.0, 0.0);

    PbdFractureSurfaceMeshBuilder builder;
    builder.updateTetCutSide(tetMesh, restPositions, *vertices, cutSurface, 4.0, 4.0, 1.0, 0.0);

    EXPECT_EQ(builder.getSurfaceMesh()->getNumVertices(), 3);
    EXPECT_EQ(builder.getSurfaceMesh()->getNumTriangles(), 1);
    const VecDataArray<double, 3>& cutVertices = *builder.getSurfaceMesh()->getVertexPositions();
    for (int i = 0; i < 3; i++)
    {
        EXPECT_GT(cutVertices[i].y(), 1.0);
    }
}

TEST(PbdFractureTearHelperTest, TetCutSurfaceCanTriangulateHorizontalQuad)
{
    auto vertices = std::make_shared<VecDataArray<double, 3>>();
    vertices->push_back(Vec3d(-1.0, -1.0, 0.0));
    vertices->push_back(Vec3d(1.0, -1.0, 0.0));
    vertices->push_back(Vec3d(0.0, 1.0, -1.0));
    vertices->push_back(Vec3d(0.0, 1.0, 1.0));

    auto cells = std::make_shared<VecDataArray<int, 4>>();
    cells->push_back(Vec4i(0, 1, 2, 3));

    auto tetMesh = std::make_shared<TetrahedralMesh>();
    tetMesh->initialize(vertices, cells);

    const std::vector<Vec3d> restPositions = { (*vertices)[0], (*vertices)[1], (*vertices)[2], (*vertices)[3] };
    TearPath path({ Vec3d(-2.0, 0.0, 0.0), Vec3d(2.0, 0.0, 0.0) });
    TearCutSurface cutSurface(path, 2.0, TearCutSurface::Mode::HorizontalLayer);

    PbdFractureSurfaceMeshBuilder builder;
    builder.updateTetCutSide(tetMesh, restPositions, *vertices, cutSurface, 4.0, 4.0, 1.0, 0.0);

    EXPECT_EQ(builder.getSurfaceMesh()->getNumVertices(), 4);
    EXPECT_EQ(builder.getSurfaceMesh()->getNumTriangles(), 2);
}

TEST(PbdFractureTearHelperTest, TetCutSurfaceSnapsToRequestedTetLayer)
{
    auto vertices = std::make_shared<VecDataArray<double, 3>>();
    vertices->push_back(Vec3d(-1.0, 0.0, -1.0));
    vertices->push_back(Vec3d(1.0, 0.0, -1.0));
    vertices->push_back(Vec3d(0.0, 1.0, -1.0));
    vertices->push_back(Vec3d(0.0, 1.0, 1.0));

    auto cells = std::make_shared<VecDataArray<int, 4>>();
    cells->push_back(Vec4i(0, 1, 2, 3));

    auto tetMesh = std::make_shared<TetrahedralMesh>();
    tetMesh->initialize(vertices, cells);

    const std::vector<Vec3d> restPositions = { (*vertices)[0], (*vertices)[1], (*vertices)[2], (*vertices)[3] };
    TearPath path({ Vec3d(-2.0, 0.25, 0.0), Vec3d(2.0, 0.25, 0.0) });
    TearCutSurface cutSurface(path, 2.0, TearCutSurface::Mode::HorizontalLayer);

    PbdFractureSurfaceMeshBuilder upperBuilder;
    PbdFractureSurfaceMeshBuilder lowerBuilder;
    upperBuilder.updateTetCutSide(tetMesh, restPositions, *vertices, cutSurface, 4.0, 4.0, 1.0, 0.0);
    lowerBuilder.updateTetCutSide(tetMesh, restPositions, *vertices, cutSurface, 4.0, 4.0, -1.0, 0.0);

    const VecDataArray<double, 3>& upperVertices = *upperBuilder.getSurfaceMesh()->getVertexPositions();
    const VecDataArray<double, 3>& lowerVertices = *lowerBuilder.getSurfaceMesh()->getVertexPositions();
    ASSERT_EQ(upperVertices.size(), 4);
    ASSERT_EQ(lowerVertices.size(), 4);
    for (int i = 0; i < 4; i++)
    {
        EXPECT_DOUBLE_EQ(upperVertices[i].y(), 1.0);
        EXPECT_DOUBLE_EQ(lowerVertices[i].y(), 0.0);
    }
}
