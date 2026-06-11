/*
** This file is part of the Interactive Medical Simulation Toolkit (iMSTK)
** iMSTK is distributed under the Apache License, Version 2.0.
** See accompanying NOTICE for details.
*/

#pragma once

#include "imstkLineMesh.h"
#include "imstkSurfaceMesh.h"
#include "imstkTetrahedralMesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace imstk
{
struct TearPathSample
{
    Vec3d  point = Vec3d::Zero();
    Vec3d  tangent = Vec3d::UnitX();
    double distanceAlongPath = 0.0;
    double distanceToCurtain = std::numeric_limits<double>::max();
};

class TearPath
{
public:
    TearPath() = default;

    explicit TearPath(const std::vector<Vec3d>& points)
    {
        setPoints(points);
    }

    void setPoints(const std::vector<Vec3d>& points)
    {
        m_points = points;
        rebuildLengths();
    }

    const std::vector<Vec3d>& getPoints() const { return m_points; }
    double getLength() const { return m_totalLength; }

    Vec3d sample(const double distance) const
    {
        if (m_points.empty())
        {
            return Vec3d::Zero();
        }
        if (m_points.size() == 1 || distance <= 0.0)
        {
            return m_points.front();
        }
        if (distance >= m_totalLength)
        {
            return m_points.back();
        }

        for (size_t i = 0; i + 1 < m_points.size(); i++)
        {
            if (distance <= m_cumulativeLengths[i + 1])
            {
                const double segmentStart = m_cumulativeLengths[i];
                const double segmentLength = m_cumulativeLengths[i + 1] - segmentStart;
                const double t = (segmentLength > 0.0) ? (distance - segmentStart) / segmentLength : 0.0;
                return (1.0 - t) * m_points[i] + t * m_points[i + 1];
            }
        }

        return m_points.back();
    }

    std::vector<Vec3d> getSamplesUpTo(const double frontDistance) const
    {
        std::vector<Vec3d> samples;
        if (m_points.size() < 2 || frontDistance <= 0.0)
        {
            return samples;
        }

        const double clampedFront = std::min(frontDistance, m_totalLength);
        samples.push_back(m_points.front());
        for (size_t i = 1; i < m_points.size(); i++)
        {
            if (m_cumulativeLengths[i] < clampedFront)
            {
                samples.push_back(m_points[i]);
            }
        }

        const Vec3d frontPoint = sample(clampedFront);
        if ((frontPoint - samples.back()).norm() > 1e-8)
        {
            samples.push_back(frontPoint);
        }

        return samples;
    }

    TearPathSample getClosestSampleOnCurtain(const Vec3d& point, const double halfThickness) const
    {
        TearPathSample result;
        if (m_points.size() < 2)
        {
            return result;
        }

        double bestSquaredDistance = std::numeric_limits<double>::max();
        const double yMin = -halfThickness;
        const double yMax = halfThickness;

        for (size_t i = 0; i + 1 < m_points.size(); i++)
        {
            const Vec3d p0 = m_points[i];
            const Vec3d p1 = m_points[i + 1];
            const Vec3d segment = p1 - p0;
            const Vec2d segmentXZ(segment.x(), segment.z());
            const Vec2d pointXZ(point.x() - p0.x(), point.z() - p0.z());
            const double segmentLengthSquared = segmentXZ.squaredNorm();
            const double t = (segmentLengthSquared > 0.0) ?
                std::max(0.0, std::min(1.0, pointXZ.dot(segmentXZ) / segmentLengthSquared)) : 0.0;

            const Vec3d pathPoint = p0 + t * segment;
            const double clampedY = std::max(yMin, std::min(yMax, point.y()));
            const Vec3d curtainPoint(pathPoint.x(), clampedY, pathPoint.z());
            const double squaredDistance = (point - curtainPoint).squaredNorm();

            if (squaredDistance < bestSquaredDistance)
            {
                bestSquaredDistance = squaredDistance;
                result.point = pathPoint;
                result.tangent = segment.norm() > 0.0 ? segment.normalized() : Vec3d::UnitX();
                result.distanceAlongPath =
                    m_cumulativeLengths[i] + t * (m_cumulativeLengths[i + 1] - m_cumulativeLengths[i]);
                result.distanceToCurtain = std::sqrt(squaredDistance);
            }
        }

        return result;
    }

    TearPathSample getClosestSampleOnPathXZ(const Vec3d& point) const
    {
        TearPathSample result;
        if (m_points.size() < 2)
        {
            return result;
        }

        double bestSquaredDistance = std::numeric_limits<double>::max();
        for (size_t i = 0; i + 1 < m_points.size(); i++)
        {
            const Vec3d p0 = m_points[i];
            const Vec3d p1 = m_points[i + 1];
            const Vec3d segment = p1 - p0;
            const Vec2d segmentXZ(segment.x(), segment.z());
            const Vec2d pointXZ(point.x() - p0.x(), point.z() - p0.z());
            const double segmentLengthSquared = segmentXZ.squaredNorm();
            const double t = (segmentLengthSquared > 0.0) ?
                std::max(0.0, std::min(1.0, pointXZ.dot(segmentXZ) / segmentLengthSquared)) : 0.0;

            const Vec3d pathPoint = p0 + t * segment;
            const Vec2d deltaXZ(point.x() - pathPoint.x(), point.z() - pathPoint.z());
            const double squaredDistance = deltaXZ.squaredNorm();
            if (squaredDistance < bestSquaredDistance)
            {
                bestSquaredDistance = squaredDistance;
                result.point = pathPoint;
                result.tangent = segment.norm() > 0.0 ? segment.normalized() : Vec3d::UnitX();
                result.distanceAlongPath =
                    m_cumulativeLengths[i] + t * (m_cumulativeLengths[i + 1] - m_cumulativeLengths[i]);
                result.distanceToCurtain = std::sqrt(squaredDistance);
            }
        }

        return result;
    }

private:
    void rebuildLengths()
    {
        m_cumulativeLengths.clear();
        m_totalLength = 0.0;
        if (m_points.empty())
        {
            return;
        }

        m_cumulativeLengths.reserve(m_points.size());
        m_cumulativeLengths.push_back(0.0);
        for (size_t i = 1; i < m_points.size(); i++)
        {
            m_totalLength += (m_points[i] - m_points[i - 1]).norm();
            m_cumulativeLengths.push_back(m_totalLength);
        }
    }

    std::vector<Vec3d> m_points;
    std::vector<double> m_cumulativeLengths;
    double m_totalLength = 0.0;
};

struct TearCutSurfaceSample
{
    Vec3d  point = Vec3d::Zero();
    Vec3d  tangent = Vec3d::UnitX();
    Vec3d  normal = Vec3d::UnitY();
    double distanceAlongPath = 0.0;
    double distanceToSurface = std::numeric_limits<double>::max();
};

class TearCutSurface
{
public:
    enum class Mode
    {
        VerticalCurtain,
        HorizontalLayer
    };

    TearCutSurface() = default;

    TearCutSurface(const TearPath& path, const double halfThickness, const Mode mode) :
        m_path(path),
        m_halfThickness(halfThickness),
        m_mode(mode)
    {
    }

    const TearPath& getPath() const { return m_path; }
    double getHalfThickness() const { return m_halfThickness; }
    Mode getMode() const { return m_mode; }

    TearCutSurfaceSample closestSample(const Vec3d& point) const
    {
        if (m_mode == Mode::HorizontalLayer)
        {
            const TearPathSample pathSample = m_path.getClosestSampleOnPathXZ(point);
            TearCutSurfaceSample sample;
            sample.point = pathSample.point;
            sample.tangent = pathSample.tangent;
            sample.normal = Vec3d::UnitY();
            sample.distanceAlongPath = pathSample.distanceAlongPath;
            sample.distanceToSurface = 0.0;
            return sample;
        }

        const TearPathSample pathSample = m_path.getClosestSampleOnCurtain(point, m_halfThickness);
        TearCutSurfaceSample sample;
        sample.point = pathSample.point;
        sample.tangent = pathSample.tangent;
        sample.normal = getHorizontalNormal(pathSample.tangent);
        sample.distanceAlongPath = pathSample.distanceAlongPath;
        sample.distanceToSurface = pathSample.distanceToCurtain;
        return sample;
    }

    double signedDistance(const Vec3d& point) const
    {
        const TearCutSurfaceSample sample = closestSample(point);
        return (point - sample.point).dot(sample.normal);
    }

private:
    static Vec3d getHorizontalNormal(const Vec3d& tangent)
    {
        Vec3d normal(-tangent.z(), 0.0, tangent.x());
        if (normal.norm() < 1e-8)
        {
            return Vec3d::UnitZ();
        }

        return normal.normalized();
    }

    TearPath m_path;
    double m_halfThickness = 1.0;
    Mode m_mode = Mode::VerticalCurtain;
};

struct TearState
{
    bool   active = false;
    double frontDistance = 0.0;
    double speed = 2.0;
    double tearRadius = 0.42;
    double halfThickness = 1.05;
};

class PbdFractureSurfaceMeshBuilder
{
public:
    PbdFractureSurfaceMeshBuilder()
    {
        m_surfaceMesh = std::make_shared<SurfaceMesh>();
        m_vertices    = std::make_shared<VecDataArray<double, 3>>();
        m_triangles   = std::make_shared<VecDataArray<int, 3>>();
        m_surfaceMesh->initialize(m_vertices, m_triangles);
    }

    std::shared_ptr<SurfaceMesh> getSurfaceMesh() const { return m_surfaceMesh; }

    void update(const TearPath& path, const double frontDistance, const double halfThickness)
    {
        m_vertices->clear();
        m_triangles->clear();

        const std::vector<Vec3d> samples = path.getSamplesUpTo(frontDistance);
        if (samples.size() >= 2)
        {
            m_vertices->reserve(samples.size() * 2);
            m_triangles->reserve((samples.size() - 1) * 2);

            const Vec3d up(0.0, halfThickness, 0.0);
            for (const Vec3d& sample : samples)
            {
                m_vertices->push_back(sample - up);
                m_vertices->push_back(sample + up);
            }

            for (int i = 0; i + 1 < static_cast<int>(samples.size()); i++)
            {
                const int lower0 = i * 2;
                const int upper0 = lower0 + 1;
                const int lower1 = lower0 + 2;
                const int upper1 = lower0 + 3;
                m_triangles->push_back(Vec3i(lower0, lower1, upper1));
                m_triangles->push_back(Vec3i(lower0, upper1, upper0));
            }
        }

        m_vertices->postModified();
        m_triangles->postModified();
        m_surfaceMesh->postModified();
    }

    void updateSeparatedSides(const TearPath& path,
                              const double frontDistance,
                              const double halfThickness,
                              const Vec3d& liftedOffset,
                              const double sideGap = 0.025)
    {
        m_vertices->clear();
        m_triangles->clear();

        const std::vector<Vec3d> samples = path.getSamplesUpTo(frontDistance);
        if (samples.size() >= 2)
        {
            m_vertices->reserve(samples.size() * 4);
            m_triangles->reserve((samples.size() - 1) * 4);

            const Vec3d up(0.0, halfThickness, 0.0);
            for (int i = 0; i < static_cast<int>(samples.size()); i++)
            {
                Vec3d tangent = Vec3d::UnitX();
                if (i + 1 < static_cast<int>(samples.size()))
                {
                    tangent = samples[i + 1] - samples[i];
                }
                else
                {
                    tangent = samples[i] - samples[i - 1];
                }

                Vec3d normal(-tangent.z(), 0.0, tangent.x());
                if (normal.norm() < 1e-8)
                {
                    normal = Vec3d::UnitZ();
                }
                else
                {
                    normal.normalize();
                }

                const Vec3d stationaryShift = -normal * sideGap;
                const Vec3d liftedShift = liftedOffset + normal * sideGap;
                m_vertices->push_back(samples[i] - up + stationaryShift);
                m_vertices->push_back(samples[i] + up + stationaryShift);
                m_vertices->push_back(samples[i] - up + liftedShift);
                m_vertices->push_back(samples[i] + up + liftedShift);
            }

            for (int i = 0; i + 1 < static_cast<int>(samples.size()); i++)
            {
                const int stationaryLower0 = i * 4;
                const int stationaryUpper0 = stationaryLower0 + 1;
                const int liftedLower0 = stationaryLower0 + 2;
                const int liftedUpper0 = stationaryLower0 + 3;
                const int stationaryLower1 = stationaryLower0 + 4;
                const int stationaryUpper1 = stationaryLower0 + 5;
                const int liftedLower1 = stationaryLower0 + 6;
                const int liftedUpper1 = stationaryLower0 + 7;

                m_triangles->push_back(Vec3i(stationaryLower0, stationaryUpper1, stationaryLower1));
                m_triangles->push_back(Vec3i(stationaryLower0, stationaryUpper0, stationaryUpper1));
                m_triangles->push_back(Vec3i(liftedLower0, liftedLower1, liftedUpper1));
                m_triangles->push_back(Vec3i(liftedLower0, liftedUpper1, liftedUpper0));
            }
        }

        m_vertices->postModified();
        m_triangles->postModified();
        m_surfaceMesh->postModified();
    }

    void updateSide(const TearPath& path,
                    const double frontDistance,
                    const double halfThickness,
                    const Vec3d& sideOffset,
                    const double sideGap)
    {
        m_vertices->clear();
        m_triangles->clear();

        const std::vector<Vec3d> samples = path.getSamplesUpTo(frontDistance);
        if (samples.size() >= 2)
        {
            m_vertices->reserve(samples.size() * 2);
            m_triangles->reserve((samples.size() - 1) * 2);

            const Vec3d up(0.0, halfThickness, 0.0);
            for (int i = 0; i < static_cast<int>(samples.size()); i++)
            {
                Vec3d tangent = Vec3d::UnitX();
                if (i + 1 < static_cast<int>(samples.size()))
                {
                    tangent = samples[i + 1] - samples[i];
                }
                else
                {
                    tangent = samples[i] - samples[i - 1];
                }

                Vec3d normal(-tangent.z(), 0.0, tangent.x());
                if (normal.norm() < 1e-8)
                {
                    normal = Vec3d::UnitZ();
                }
                else
                {
                    normal.normalize();
                }

                const Vec3d shift = sideOffset + normal * sideGap;
                m_vertices->push_back(samples[i] - up + shift);
                m_vertices->push_back(samples[i] + up + shift);
            }

            for (int i = 0; i + 1 < static_cast<int>(samples.size()); i++)
            {
                const int lower0 = i * 2;
                const int upper0 = lower0 + 1;
                const int lower1 = lower0 + 2;
                const int upper1 = lower0 + 3;
                m_triangles->push_back(Vec3i(lower0, lower1, upper1));
                m_triangles->push_back(Vec3i(lower0, upper1, upper0));
            }
        }

        m_vertices->postModified();
        m_triangles->postModified();
        m_surfaceMesh->postModified();
    }

    void updateTetCutSide(const std::shared_ptr<TetrahedralMesh> tetMesh,
                          const std::vector<Vec3d>& restPositions,
                          const VecDataArray<double, 3>& currentPositions,
                          const TearCutSurface& cutSurface,
                          const double frontDistance,
                          const double tearRadius,
                          const double sideSign,
                          const double sideGap)
    {
        m_vertices->clear();
        m_triangles->clear();

        if (tetMesh == nullptr || frontDistance <= 0.0
            || restPositions.size() < static_cast<size_t>(tetMesh->getNumVertices())
            || currentPositions.size() < tetMesh->getNumVertices())
        {
            postModified();
            return;
        }

        const VecDataArray<int, 4>& tets = *tetMesh->getCells();
        m_vertices->reserve(tets.size() * 4);
        m_triangles->reserve(tets.size() * 2);

        constexpr std::array<std::array<int, 2>, 6> tetEdges = {
            std::array<int, 2>{ 0, 1 },
            std::array<int, 2>{ 0, 2 },
            std::array<int, 2>{ 0, 3 },
            std::array<int, 2>{ 1, 2 },
            std::array<int, 2>{ 1, 3 },
            std::array<int, 2>{ 2, 3 }
        };
        constexpr double sideEpsilon = 1e-8;

        struct CutVertex
        {
            Vec3d position = Vec3d::Zero();
            Vec3d restPosition = Vec3d::Zero();
            int sideVertexId = -1;
        };

        for (int tetIndex = 0; tetIndex < tets.size(); tetIndex++)
        {
            const Vec4i& tet = tets[tetIndex];
            std::array<double, 4> side = { 0.0, 0.0, 0.0, 0.0 };
            for (int corner = 0; corner < 4; corner++)
            {
                side[corner] = cutSurface.signedDistance(restPositions[tet[corner]]);
            }

            std::vector<CutVertex> cutVertices;
            cutVertices.reserve(4);
            for (const std::array<int, 2>& edge : tetEdges)
            {
                const int localA = edge[0];
                const int localB = edge[1];
                const double sideA = side[localA];
                const double sideB = side[localB];
                if (sideA * sideB >= -sideEpsilon)
                {
                    continue;
                }

                const double t = std::abs(sideA) / (std::abs(sideA) + std::abs(sideB));
                const int globalA = tet[localA];
                const int globalB = tet[localB];
                const Vec3d restCutPosition =
                    (1.0 - t) * restPositions[globalA] + t * restPositions[globalB];
                const int sideLocal = (side[localA] * sideSign >= 0.0) ? localA : localB;

                CutVertex cutVertex;
                cutVertex.restPosition = restCutPosition;
                cutVertex.sideVertexId = tet[sideLocal];
                cutVertices.push_back(cutVertex);
            }

            if (cutVertices.size() < 3 || cutVertices.size() > 4)
            {
                continue;
            }

            Vec3d restCentroid = Vec3d::Zero();
            for (const CutVertex& cutVertex : cutVertices)
            {
                restCentroid += cutVertex.restPosition;
            }
            restCentroid /= static_cast<double>(cutVertices.size());

            const TearCutSurfaceSample sample = cutSurface.closestSample(restCentroid);
            const Vec3d normal = sample.normal;

            for (CutVertex& cutVertex : cutVertices)
            {
                const int sideGlobal = cutVertex.sideVertexId;
                const Vec3d restOffset = cutVertex.restPosition - restPositions[sideGlobal];
                const Vec3d tangentialOffset = restOffset - normal * restOffset.dot(normal);
                cutVertex.position = currentPositions[sideGlobal] + tangentialOffset;
            }

            bool isNearActiveFront = false;
            for (const CutVertex& cutVertex : cutVertices)
            {
                const TearCutSurfaceSample vertexSample = cutSurface.closestSample(cutVertex.restPosition);
                if (vertexSample.distanceAlongPath <= frontDistance + tearRadius
                    && vertexSample.distanceToSurface <= tearRadius * 1.8)
                {
                    isNearActiveFront = true;
                    break;
                }
            }
            if (!isNearActiveFront)
            {
                continue;
            }

            Vec3d axisU = sample.tangent - normal * sample.tangent.dot(normal);
            if (axisU.norm() < 1e-8)
            {
                const Vec3d reference = std::abs(normal.dot(Vec3d::UnitY())) < 0.9 ?
                    Vec3d::UnitY() : Vec3d::UnitX();
                axisU = reference - normal * reference.dot(normal);
            }
            axisU.normalize();
            Vec3d axisV = normal.cross(axisU);
            if (axisV.norm() < 1e-8)
            {
                axisV = Vec3d::UnitZ();
            }
            else
            {
                axisV.normalize();
            }

            std::sort(cutVertices.begin(), cutVertices.end(),
                [&](const CutVertex& lhs, const CutVertex& rhs)
                {
                    const Vec3d left = lhs.restPosition - restCentroid;
                    const Vec3d right = rhs.restPosition - restCentroid;
                    const double leftAngle = std::atan2(left.dot(axisV), left.dot(axisU));
                    const double rightAngle = std::atan2(right.dot(axisV), right.dot(axisU));
                    return leftAngle < rightAngle;
                });

            const int firstVertex = static_cast<int>(m_vertices->size());
            for (const CutVertex& cutVertex : cutVertices)
            {
                m_vertices->push_back(cutVertex.position + normal * sideGap);
            }

            if (sideSign >= 0.0)
            {
                m_triangles->push_back(Vec3i(firstVertex, firstVertex + 1, firstVertex + 2));
                if (cutVertices.size() == 4)
                {
                    m_triangles->push_back(Vec3i(firstVertex, firstVertex + 2, firstVertex + 3));
                }
            }
            else
            {
                m_triangles->push_back(Vec3i(firstVertex + 2, firstVertex + 1, firstVertex));
                if (cutVertices.size() == 4)
                {
                    m_triangles->push_back(Vec3i(firstVertex + 3, firstVertex + 2, firstVertex));
                }
            }
        }

        postModified();
    }

private:
    void postModified()
    {
        m_vertices->postModified();
        m_triangles->postModified();
        m_surfaceMesh->postModified();
    }

    std::shared_ptr<SurfaceMesh> m_surfaceMesh;
    std::shared_ptr<VecDataArray<double, 3>> m_vertices;
    std::shared_ptr<VecDataArray<int, 3>> m_triangles;
};
} // namespace imstk
