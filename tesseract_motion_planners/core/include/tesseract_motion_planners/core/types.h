/**
 * @file planner_types.h
 * @brief Planner types.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_MOTION_PLANNERS_PLANNER_TYPES_H
#define TESSERACT_MOTION_PLANNERS_PLANNER_TYPES_H

#include <tesseract_environment/environment.h>
#include <tesseract_common/types.h>
#include <tesseract_command_language/poly/instruction_poly.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>

#include <tesseract_motion_planners/core/profile_dictionary.h>

namespace tesseract_planning
{
struct PlannerRequest
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  /** @brief The name of the process manager to use */
  std::string name;

  /** @brief The environment */
  tesseract_environment::Environment::ConstPtr env;

  /** @brief The start state to use for planning */
  tesseract_scene_graph::SceneState env_state;

  /** @brief The profile dictionary */
  ProfileDictionary::ConstPtr profiles{ std::make_shared<ProfileDictionary>() };

  tmp::WaypointProfileDictionary::ConstPtr waypoint_profiles{ std::make_shared<tmp::WaypointProfileDictionary>() };
  tmp::CompositeProfileDictionary::ConstPtr composite_profiles{ std::make_shared<tmp::CompositeProfileDictionary>() };
  tmp::PlannerProfileDictionary::ConstPtr planner_profiles{ std::make_shared<tmp::PlannerProfileDictionary>() };

  /**
   * @brief The program instruction
   * This must contain a minimum of two move instruction the first move instruction is the start state
   */
  CompositeInstruction instructions;

  /** @brief Indicate if output should be verbose */
  bool verbose{ false };

  /**
   * @brief Format the result as input for motion planning
   * @details
   *    - If true it uses the input waypoint but updates the seed component
   *    - If false, it replace the input waypoint with a state waypoint
   */
  bool format_result_as_input{ false };

  /**
   * @brief data Planner specific data. For planners included in Tesseract_planning this is the planner problem that
   * will be used if it is not null
   */
  std::shared_ptr<void> data;
};

struct PlannerResponse
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  CompositeInstruction results;
  /** @brief Indicate if planning was successful */
  bool successful{ false };
  /** @brief The status message */
  std::string message;
  /** @brief Waypoints for which the planner succeeded */
  std::vector<std::reference_wrapper<InstructionPoly>> succeeded_instructions{};
  /** @brief Waypoints for which the planner failed */
  std::vector<std::reference_wrapper<InstructionPoly>> failed_instructions{};
  /** @brief Planner specific data. Planners in Tesseract_planning use this to store the planner problem that was solved
   */
  std::shared_ptr<void> data;

  /** @brief This return true if successful */
  explicit operator bool() const noexcept { return successful; }
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PLANNING_PLANNER_TYPES_H
