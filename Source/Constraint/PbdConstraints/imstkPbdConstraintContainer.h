/*
** This file is part of the Interactive Medical Simulation Toolkit (iMSTK)
** iMSTK is distributed under the Apache License, Version 2.0.
** See accompanying NOTICE for details.
*/

#pragma once

#include "imstkPbdConstraint.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace imstk
{
///
/// \class PbdConstraintContainer
///
/// \brief Container for pbd constraints
///
class PbdConstraintContainer
{
public:
    PbdConstraintContainer() = default;
    virtual ~PbdConstraintContainer() = default;

public:
    using iterator       = std::vector<std::shared_ptr<PbdConstraint>>::iterator;
    using const_iterator = std::vector<std::shared_ptr<PbdConstraint>>::const_iterator;

    ///
    /// \brief Unique key for edge constraints within one PBD body
    ///
    struct EdgeKey
    {
        EdgeKey() = default;
        EdgeKey(const int bodyId, const int i0, const int i1) :
            bodyId(bodyId),
            vertexA(std::min(i0, i1)),
            vertexB(std::max(i0, i1))
        {
        }

        bool operator==(const EdgeKey& other) const
        {
            return bodyId == other.bodyId && vertexA == other.vertexA && vertexB == other.vertexB;
        }

        int bodyId  = -1;
        int vertexA = -1;
        int vertexB = -1;
    };

public:
    ///
    /// \brief Adds a constraint to the system, thread safe
    ///
    virtual void addConstraint(std::shared_ptr<PbdConstraint> constraint);

    ///
    /// \brief Adds a constraint and indexes it against an edge in a PBD body
    ///
    virtual void addConstraintForEdge(std::shared_ptr<PbdConstraint> constraint, int bodyId, int i0, int i1);

    ///
    /// \brief Gets all constraints indexed against an edge in a PBD body
    ///
    virtual std::vector<std::shared_ptr<PbdConstraint>> getConstraintsForEdge(int bodyId, int i0, int i1) const;

    ///
    /// \brief Deactivates all active constraints indexed against an edge in a PBD body
    /// \return number of constraints newly deactivated
    ///
    virtual size_t deactivateConstraintsForEdge(int bodyId, int i0, int i1);

    ///
    /// \brief Linear searches for and removes a constraint from the system, thread safe
    ///
    virtual void removeConstraint(std::shared_ptr<PbdConstraint> constraint);

    ///
    /// \brief Removes all constraints associated with vertex ids
    ///
    virtual void removeConstraints(
        std::shared_ptr<std::unordered_set<size_t>> vertices, const int bodyId);

    ///
    /// \brief Removes a constraint from the system by iterator, thread safe
    ///
    virtual iterator eraseConstraint(iterator iter);
    virtual const_iterator eraseConstraint(const_iterator iter);

    ///
    /// \brief Reserve an amount of constraints in the pool, if you know
    /// ahead of time the number of constraints, or even an estimate, it
    /// can be faster to first reserve them
    ///
    virtual void reserve(const size_t n) { m_constraints.reserve(n); }

    ///
    /// \brief Returns if there are no constraints
    ///
    const bool empty() const { return m_constraints.empty() && m_partitionedConstraints.empty(); }

    ///
    /// \brief Get the underlying container
    ///
    const std::vector<std::shared_ptr<PbdConstraint>>& getConstraints() const { return m_constraints; }
    std::vector<std::shared_ptr<PbdConstraint>>& getConstraints() { return m_constraints; }

    ///
    /// \brief Get the partitioned constraints
    ///
    const std::vector<std::vector<std::shared_ptr<PbdConstraint>>>& getPartitionedConstraints() const { return m_partitionedConstraints; }

    ///
    /// \brief Partitions pbd constraints into separate vectors via graph coloring
    /// \param Minimum number of constraints in groups, any under will be dumped back into m_constraints
    ///
    void partitionConstraints(const int partitionThreshold);

    ///
    /// \brief Clear the parition vectors
    ///
    void clearPartitions() { m_partitionedConstraints.clear(); }

protected:
    ///
    /// \brief Removes expired weak references and constraints matching the predicate from the edge map
    ///
    void compactEdgeConstraintMap(const std::function<bool(const std::shared_ptr<PbdConstraint>&)>& removeConstraintFunc);

    struct EdgeKeyHash
    {
        size_t operator()(const EdgeKey& key) const
        {
            size_t seed = static_cast<size_t>(key.bodyId);
            seed ^= static_cast<size_t>(key.vertexA) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= static_cast<size_t>(key.vertexB) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    std::vector<std::shared_ptr<PbdConstraint>> m_constraints;                         ///< Not partitioned constraints
    std::vector<std::vector<std::shared_ptr<PbdConstraint>>> m_partitionedConstraints; ///< Partitioned pbd constraints
    std::unordered_map<EdgeKey, std::vector<std::weak_ptr<PbdConstraint>>, EdgeKeyHash> m_edgeConstraints;
    mutable ParallelUtils::SpinLock m_constraintLock;                                  ///< Used to deal with concurrent addition/removal of constraints
};
} // namespace imstk
