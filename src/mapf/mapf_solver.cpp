#include "mapf_solver.h"
#include "mapf_decoupled.h"
#include "mapf_cbs.h"
#include <stdexcept>
#include <algorithm>

// ============================================================================
// Helper Function: Compute Obstacle Volume Percentage
// ============================================================================

double MAPFSolver::computeObstacleVolumePercent(
    const ob::RealVectorBounds& region_bounds,
    const std::vector<fcl::CollisionObjectf*>& obstacles)
{
    // Compute region volume (2D area for 2D problems)
    double region_width = region_bounds.high[0] - region_bounds.low[0];
    double region_height = region_bounds.high[1] - region_bounds.low[1];
    double region_volume = region_width * region_height;

    if (region_volume <= 0.0) {
        return 0.0;
    }

    // Create region AABB
    fcl::Vector3f region_min(region_bounds.low[0], region_bounds.low[1], -1.0f);
    fcl::Vector3f region_max(region_bounds.high[0], region_bounds.high[1], 1.0f);

    double total_obstacle_volume = 0.0;

    // For each obstacle, compute intersection volume with region
    for (const auto* obstacle : obstacles) {
        // Get obstacle AABB
        const fcl::AABBf& obs_aabb = obstacle->getAABB();

        // Compute intersection in X dimension
        float x_min = std::max(region_min[0], obs_aabb.min_[0]);
        float x_max = std::min(region_max[0], obs_aabb.max_[0]);

        // Compute intersection in Y dimension
        float y_min = std::max(region_min[1], obs_aabb.min_[1]);
        float y_max = std::min(region_max[1], obs_aabb.max_[1]);

        // Check if there's an intersection
        if (x_max > x_min && y_max > y_min) {
            double intersection_volume = (x_max - x_min) * (y_max - y_min);
            total_obstacle_volume += intersection_volume;
        }
    }

    // Return percentage (0.0 to 1.0)
    return total_obstacle_volume / region_volume;
}

// ============================================================================
// Factory Function
// ============================================================================

std::unique_ptr<MAPFSolver> createMAPFSolver(
    const std::string& method,
    int region_capacity,
    double timeout)
{
    if (method == "decoupled" || method == "astar") {
        return std::make_unique<DecoupledMAPFSolver>();
    }
    else if (method == "cbs") {
        return std::make_unique<CBSMAPFSolver>(region_capacity, timeout);
    }

    throw std::runtime_error("Unknown MAPF method: " + method);
}
