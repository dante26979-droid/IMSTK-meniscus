/*
** This file is part of the Interactive Medical Simulation Toolkit (iMSTK)
** iMSTK is distributed under the Apache License, Version 2.0.
** See accompanying NOTICE for details.
*/

#include "imstkPbdSolver.h"
#include "imstkLogger.h"
#include "imstkParallelUtils.h"
#include "imstkPbdCollisionConstraint.h"
#include "imstkPbdConstraintContainer.h"

namespace imstk
{
PbdSolver::PbdSolver() :
    m_constraints(std::make_shared<PbdConstraintContainer>()),
    m_constraintLists(std::make_shared<std::list<std::vector<PbdConstraint*>*>>())
{
}

void
PbdSolver::solve()
{
    if (m_dataTracker)
    {
        m_dataTracker->getStopWatch(DataTracker::ePhysics::SolverTime_ms).start();
    }

    size_t                                                          numConstraints = 0;
    const std::vector<std::shared_ptr<PbdConstraint>>&              constraints    = m_constraints->getConstraints();
    const std::vector<std::vector<std::shared_ptr<PbdConstraint>>>& partitionedConstraints = m_constraints->getPartitionedConstraints();

    double averageC      = 0.0;
    double averageLambda = 0.0;
    // Zero out the Lagrange multiplier
    for (const auto& constraint : constraints)
    {
        if (constraint->isActive())
        {
            numConstraints++;
            constraint->zeroOutLambda();
        }
    }

    // Zero out paritioned constraints
    for (const auto& constraintPartition : partitionedConstraints)
    {
        ParallelUtils::parallelFor(constraintPartition.size(),
            [&](const size_t idx)
            {
                if (constraintPartition[idx]->isActive())
                {
                    constraintPartition[idx]->zeroOutLambda();
                }
            });
        for (const auto& constraint : constraintPartition)
        {
            if (constraint->isActive())
            {
                numConstraints++;
            }
        }
    }

    // Zero out insertion/collision constraints
    for (auto constraintList : *m_constraintLists)
    {
        const std::vector<PbdConstraint*>& constraintVec = *constraintList;
        for (size_t j = 0; j < constraintVec.size(); j++)
        {
            if (constraintVec[j]->isActive())
            {
                numConstraints++;
                constraintVec[j]->zeroOutLambda();
            }
        }
    }

    unsigned int i = 0;
    while (i++ < m_iterations)
    {
        // Project collision and all external constraints
        for (auto constraintList : *m_constraintLists)
        {
            const std::vector<PbdConstraint*>& constraintVec = *constraintList;
            for (size_t j = 0; j < constraintVec.size(); j++)
            {
                if (constraintVec[j]->isActive())
                {
                    constraintVec[j]->projectConstraint(*m_state, m_dt, m_solverType);
                }
            }
        }

        // Project all internal body constraints
        for (const auto& constraint : constraints)
        {
            if (constraint->isActive())
            {
                constraint->projectConstraint(*m_state, m_dt, m_solverType);
            }
        }

        for (const auto& constraintPartition : partitionedConstraints)
        {
            ParallelUtils::parallelFor(constraintPartition.size(),
                [&](const size_t idx)
                {
                    if (constraintPartition[idx]->isActive())
                    {
                        constraintPartition[idx]->projectConstraint(*m_state, m_dt, m_solverType);
                    }
                });
            //// Sequential
            //for (size_t k = 0; k < constraintPartition.size(); k++)
            //{
            //    constraintPartition[k]->projectConstraint(invMasses, m_dt, m_solverType, currPositions);
            //}
        }
    }

    if (m_dataTracker)
    {
        m_dataTracker->probeElapsedTime_s(DataTracker::ePhysics::SolverTime_ms);
        m_dataTracker->probe(DataTracker::ePhysics::NumConstraints, numConstraints);

        for (const auto& constraint : constraints)
        {
            if (constraint->isActive())
            {
                averageC      += constraint->getConstraintC();
                averageLambda += constraint->getLambda();
            }
        }

        for (const auto& constraintPartition : partitionedConstraints)
        {
            for (size_t k = 0; k < constraintPartition.size(); k++)
            {
                if (constraintPartition[k]->isActive())
                {
                    averageC      += constraintPartition[k]->getConstraintC();
                    averageLambda += constraintPartition[k]->getLambda();
                }
            }
        }

        for (auto constraintList : *m_constraintLists)
        {
            const std::vector<PbdConstraint*>& constraintVec = *constraintList;
            for (size_t j = 0; j < constraintVec.size(); j++)
            {
                if (constraintVec[j]->isActive())
                {
                    averageC      += constraintVec[j]->getConstraintC();
                    averageLambda += constraintVec[j]->getLambda();
                }
            }
        }

        averageC = (numConstraints > 0) ? averageC / numConstraints : 0.0;
        m_dataTracker->probe(DataTracker::ePhysics::AverageC, averageC);
    }
}
} // namespace imstk
