// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Declaration of the CostmapPlanner class implementing A* path planning.

#ifndef EASYNAV_COSTMAP_PLANNER__COSTMAPPLANNER_HPP_
#define EASYNAV_COSTMAP_PLANNER__COSTMAPPLANNER_HPP_

#include <memory>
#include <vector>

#include "nav_msgs/msg/path.hpp"

#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/// \brief A planner implementing the A* algorithm on a Costmap2D grid.
class CostmapPlanner : public PlannerMethodBase
{
public:
  /**
   * @brief Default constructor.
   *
   * Initializes the internal variables and parameters of the planner.
   */
  explicit CostmapPlanner();

  /**
   * @brief Initializes the planner.
   *
   * Configures publishers, retrieves parameters, and prepares the planner
   * for path generation using the available map data.
   *
   * @return std::expected<void, std::string> Success or an error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Updates the planner by computing a new path.
   *
   * Uses the current navigation state (including the robot's position and goal)
   * to generate a path based on the A* algorithm.
   *
   * @param nav_state The current navigation state (contains odometry and goal information).
   */
  void update(NavState & nav_state) override;

protected:
  double cost_factor_;
  double inflation_penalty_;
  double cost_axial_;
  double cost_diagonal_;

  nav_msgs::msg::Path current_path_;  ///< The last computed path.

  /// Publisher for the computed navigation path.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  /**
   * @brief Runs the A* algorithm to compute a path.
   *
   * @param map The occupancy map used for path planning.
   * @param start The starting pose in world coordinates.
   * @param goal The target pose in world coordinates.
   * @return A sequence of poses representing the planned path.
   */
  std::vector<geometry_msgs::msg::Pose> a_star_path(
    const Costmap2D & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal);

};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP_PLANNER__COSTMAPPLANNER_HPP_
