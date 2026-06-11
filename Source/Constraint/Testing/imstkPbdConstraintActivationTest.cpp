/*
** This file is part of the Interactive Medical Simulation Toolkit (iMSTK)
** iMSTK is distributed under the Apache License, Version 2.0.
** See accompanying NOTICE for details.
*/

#include "imstkPbdConstraintTest.h"
#include "imstkPbdConstraintContainer.h"
#include "imstkPbdDistanceConstraint.h"

using namespace imstk;

static std::shared_ptr<PbdDistanceConstraint>
makeDistanceConstraint(const int bodyId, const int i0, const int i1)
{
    auto constraint = std::make_shared<PbdDistanceConstraint>();
    constraint->initConstraint(1.0, { bodyId, i0 }, { bodyId, i1 }, 1.0);
    return constraint;
}

TEST_F(PbdConstraintTest, InactiveConstraintDoesNotProjectPositions)
{
    setNumParticles(2);
    m_vertices[0] = Vec3d(0.0, 0.0, 0.0);
    m_vertices[1] = Vec3d(2.0, 0.0, 0.0);
    m_invMasses[0] = 1.0;
    m_invMasses[1] = 1.0;

    PbdDistanceConstraint constraint;
    constraint.initConstraint(1.0, { 0, 0 }, { 0, 1 }, 1.0);
    constraint.setActive(false);
    m_constraint = &constraint;

    solve(0.01, PbdConstraint::SolverType::xPBD);

    EXPECT_EQ(m_vertices[0], Vec3d(0.0, 0.0, 0.0));
    EXPECT_EQ(m_vertices[1], Vec3d(2.0, 0.0, 0.0));
    EXPECT_FALSE(constraint.isActive());
}

TEST(PbdConstraintContainerTest, DeactivateConstraintsForEdgeOnlyAffectsRequestedEdge)
{
    PbdConstraintContainer container;

    std::shared_ptr<PbdDistanceConstraint> edge01 = makeDistanceConstraint(3, 0, 1);
    std::shared_ptr<PbdDistanceConstraint> edge12 = makeDistanceConstraint(3, 1, 2);
    std::shared_ptr<PbdDistanceConstraint> otherBodyEdge01 = makeDistanceConstraint(4, 0, 1);

    container.addConstraintForEdge(edge01, 3, 0, 1);
    container.addConstraintForEdge(edge12, 3, 1, 2);
    container.addConstraintForEdge(otherBodyEdge01, 4, 0, 1);

    EXPECT_EQ(container.getConstraintsForEdge(3, 0, 1).size(), 1);
    EXPECT_EQ(container.getConstraintsForEdge(3, 1, 0).size(), 1);
    EXPECT_EQ(container.getConstraintsForEdge(3, 1, 2).size(), 1);
    EXPECT_EQ(container.getConstraintsForEdge(4, 0, 1).size(), 1);

    EXPECT_EQ(container.deactivateConstraintsForEdge(3, 0, 1), 1);
    EXPECT_FALSE(edge01->isActive());
    EXPECT_TRUE(edge12->isActive());
    EXPECT_TRUE(otherBodyEdge01->isActive());
    EXPECT_EQ(container.deactivateConstraintsForEdge(3, 0, 1), 0);
}

TEST(PbdConstraintContainerTest, RemoveConstraintCleansEdgeIndex)
{
    PbdConstraintContainer container;
    std::shared_ptr<PbdDistanceConstraint> edge01 = makeDistanceConstraint(3, 0, 1);

    container.addConstraintForEdge(edge01, 3, 0, 1);
    EXPECT_EQ(container.getConstraintsForEdge(3, 0, 1).size(), 1);

    container.removeConstraint(edge01);
    EXPECT_TRUE(container.getConstraintsForEdge(3, 0, 1).empty());
}
