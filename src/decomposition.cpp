#include "decomposition.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

GridDecompositionImpl::GridDecompositionImpl(const int length, const int dim, const ompl::base::RealVectorBounds& bounds)
        : ompl::control::GridDecomposition(length, dim, bounds) {}

void GridDecompositionImpl::project(const ompl::base::State* s, std::vector<double>& coord) const
{
    if (dimension_ == 2)
    {
        coord.resize(2);
        coord[0] = s->as<ompl::base::SE2StateSpace::StateType>()->getX();
        coord[1] = s->as<ompl::base::SE2StateSpace::StateType>()->getY();
    }
    else if (dimension_ == 3)
    {
        coord.resize(3);
        coord[0] = s->as<ompl::base::SE3StateSpace::StateType>()->getX();
        coord[1] = s->as<ompl::base::SE3StateSpace::StateType>()->getY();
        coord[2] = s->as<ompl::base::SE3StateSpace::StateType>()->getZ();
    }
}

void GridDecompositionImpl::sampleFullState(const ompl::base::StateSamplerPtr& sampler, const std::vector<double>& coord, ompl::base::State* s) const
{
    sampler->sampleUniform(s);
    if (dimension_ == 2)
    {
        s->as<ompl::base::SE2StateSpace::StateType>()->setXY(coord[0], coord[1]);
    }
    else if (dimension_ == 3)
    {
        s->as<ompl::base::SE3StateSpace::StateType>()->setXYZ(coord[0], coord[1], coord[2]);
    }
}
