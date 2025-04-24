/**
 * @file raster_motion_with_approach_task.h
 * @brief Raster motion task with approach and retraction
 *
 * @author Levi Armstrong
 * @date July 29. 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#ifndef TESSERACT_TASK_COMPOSER_RASTER_MOTION_WITH_APPROACH_TASK_H
#define TESSERACT_TASK_COMPOSER_RASTER_MOTION_WITH_APPROACH_TASK_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_common/fwd.h>

namespace tesseract_planning
{
class TaskComposerPluginFactory;

/**
 * @brief The RasterMotionWithApproachTask class
 * @details The required format is below.
 *
 * Composite
 * {
 *   Composite - approach
 *   Composite - raster
 *   Composite - retraction
 * }
 */

class RasterMotionWithApproachTask : public TaskComposerTask
{
public:
  struct TaskFactoryResults
  {
    TaskComposerNode::UPtr node;
    std::string input_key;
    std::string output_key;
  };
  using TaskFactory = std::function<TaskFactoryResults(const std::string& name, std::size_t index)>;

  RasterMotionWithApproachTask();
  explicit RasterMotionWithApproachTask(std::string name,
                                        std::string input_key,
                                        std::string output_key,
                                        bool conditional,
                                        TaskFactory approach_and_retreat_task_factory,
                                        TaskFactory raster_task_factory);

  explicit RasterMotionWithApproachTask(std::string name,
                                        const YAML::Node& config,
                                        const TaskComposerPluginFactory& plugin_factory);

  ~RasterMotionWithApproachTask() override = default;
  RasterMotionWithApproachTask(const RasterMotionWithApproachTask&) = delete;
  RasterMotionWithApproachTask& operator=(const RasterMotionWithApproachTask&) = delete;
  RasterMotionWithApproachTask(RasterMotionWithApproachTask&&) = delete;
  RasterMotionWithApproachTask& operator=(RasterMotionWithApproachTask&&) = delete;

  bool operator==(const RasterMotionWithApproachTask& rhs) const;
  bool operator!=(const RasterMotionWithApproachTask& rhs) const;

protected:
  TaskFactory approach_and_retreat_task_factory_;
  TaskFactory raster_task_factory_;

  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT

  static void checkTaskInput(const tesseract_common::AnyPoly& input);

  std::unique_ptr<TaskComposerNodeInfo> runImpl(TaskComposerContext& context,
                                                OptionalTaskComposerExecutor executor) const override final;
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY2(tesseract_planning::RasterMotionWithApproachTask, "RasterMotionWithApproachTask")

#endif  // TESSERACT_TASK_COMPOSER_RASTER_MOTION_WITH_APPROACH_TASK_H
