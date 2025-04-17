/**
 * @file raster_motion_task.cpp
 * @brief Raster motion task with transitions
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <yaml-cpp/yaml.h>

#include <tesseract_common/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/planning/nodes/raster_motion_with_approach_task.h>
#include <tesseract_task_composer/planning/nodes/update_start_and_end_state_task.h>
#include <tesseract_task_composer/planning/nodes/update_end_state_task.h>
#include <tesseract_task_composer/planning/nodes/update_start_state_task.h>
#include <tesseract_task_composer/planning/nodes/motion_planner_task_info.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>

#include <tesseract_task_composer/core/nodes/start_task.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_graph.h>

#include <tesseract_command_language/composite_instruction.h>

namespace
{
tesseract_planning::RasterMotionWithApproachTask::TaskFactoryResults
createTask(const std::string& name,
           const std::string& task_name,
           const std::map<std::string, std::string>& remapping,
           const std::vector<std::string>& indexing,
           const tesseract_planning::TaskComposerPluginFactory& plugin_factory,
           std::size_t index)
{
  tesseract_planning::RasterMotionWithApproachTask::TaskFactoryResults tf_results;
  tf_results.node = plugin_factory.createTaskComposerNode(task_name);
  tf_results.node->setName(name);
  if (!remapping.empty())
  {
    tf_results.node->renameInputKeys(remapping);
    tf_results.node->renameOutputKeys(remapping);
  }

  if (!indexing.empty())
  {
    std::map<std::string, std::string> renaming;
    for (const auto& x : indexing)
    {
      std::string name = task_name;
      name.append("_");
      name.append(x);
      name.append(std::to_string(index));
      renaming[x] = name;
    }

    tf_results.node->renameInputKeys(renaming);
    tf_results.node->renameOutputKeys(renaming);
  }

  tf_results.input_key = tf_results.node->getInputKeys().front();
  tf_results.output_key = tf_results.node->getOutputKeys().front();

  return tf_results;
}
}  // namespace

namespace tesseract_planning
{
  RasterMotionWithApproachTask::RasterMotionWithApproachTask() : TaskComposerTask("RasterMotionWithApproachTask", true) {}
  RasterMotionWithApproachTask::RasterMotionWithApproachTask(std::string name,
                                   std::string input_key,
                                   std::string output_key,
                                   bool conditional,
                                   TaskFactory freespace_task_factory,
                                   TaskFactory raster_task_factory,
                                   TaskFactory transition_task_factory)
  : TaskComposerTask(std::move(name), conditional)
  , freespace_task_factory_(std::move(freespace_task_factory))
  , raster_task_factory_(std::move(raster_task_factory))
  , transition_task_factory_(std::move(transition_task_factory))
{
  input_keys_.push_back(std::move(input_key));
  output_keys_.push_back(std::move(output_key));
}

RasterMotionWithApproachTask::RasterMotionWithApproachTask(std::string name,
                                   const YAML::Node& config,
                                   const TaskComposerPluginFactory& plugin_factory)
  : TaskComposerTask(std::move(name), config)
{
  if (input_keys_.empty())
    throw std::runtime_error("RasterMotionWithApproachTask, config missing 'inputs' entry");

  if (input_keys_.size() > 1)
    throw std::runtime_error("RasterMotionWithApproachTask, config 'inputs' entry currently only supports one input key");

  if (output_keys_.empty())
    throw std::runtime_error("RasterMotionWithApproachTask, config missing 'outputs' entry");

  if (output_keys_.size() > 1)
    throw std::runtime_error("RasterMotionWithApproachTask, config 'outputs' entry currently only supports one output key");

  if (YAML::Node freespace_config = config["freespace"])
  {
    std::string task_name;
    bool has_abort_terminal_entry{ false };
    int abort_terminal_index{ -1 };
    std::vector<std::string> indexing;
    std::map<std::string, std::string> remapping;

    if (YAML::Node n = freespace_config["task"])
      task_name = n.as<std::string>();
    else
      throw std::runtime_error("RasterMotionWithApproachTask, entry 'freespace' missing 'task' entry");

    if (YAML::Node task_config = freespace_config["config"])
    {
      if (YAML::Node n = task_config["abort_terminal"])
      {
        has_abort_terminal_entry = true;
        abort_terminal_index = n.as<int>();
      }

      if (task_config["input_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, input_remapping is no longer supported use 'remapping'");

      if (task_config["output_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, output_remapping is no longer supported use 'remapping'");

      if (YAML::Node n = task_config["remapping"])
        remapping = n.as<std::map<std::string, std::string>>();

      if (task_config["input_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, input_indexing is no longer supported use 'indexing'");

      if (task_config["output_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, output_indexing is no longer supported use 'indexing'");

      if (YAML::Node n = task_config["indexing"])
        indexing = n.as<std::vector<std::string>>();
      else
        throw std::runtime_error("RasterMotionWithApproachTask, entry 'freespace' missing 'indexing' entry");
    }
    else
    {
      throw std::runtime_error("RasterMotionWithApproachTask, entry 'freespace' missing 'config' entry");
    }

    if (has_abort_terminal_entry)
    {
      freespace_task_factory_ = [task_name, abort_terminal_index, remapping, indexing, &plugin_factory](
                                    const std::string& name, std::size_t index) {
        auto tr = createTask(name, task_name, remapping, indexing, plugin_factory, index);
        static_cast<TaskComposerGraph&>(*tr.node).setTerminalTriggerAbortByIndex(abort_terminal_index);
        return tr;
      };
    }
    else
    {
      freespace_task_factory_ = [task_name, remapping, indexing, &plugin_factory](const std::string& name,
                                                                                  std::size_t index) {
        return createTask(name, task_name, remapping, indexing, plugin_factory, index);
      };
    }
  }
  else
  {
    throw std::runtime_error("RasterMotionWithApproachTask: missing 'freespace' entry");
  }

  if (YAML::Node raster_config = config["raster"])
  {
    std::string task_name;
    bool has_abort_terminal_entry{ false };
    int abort_terminal_index{ -1 };
    std::vector<std::string> indexing;
    std::map<std::string, std::string> remapping;

    if (YAML::Node n = raster_config["task"])
      task_name = n.as<std::string>();
    else
      throw std::runtime_error("RasterMotionWithApproachTask, entry 'raster' missing 'task' entry");

    if (YAML::Node task_config = raster_config["config"])
    {
      if (YAML::Node n = task_config["abort_terminal"])
      {
        has_abort_terminal_entry = true;
        abort_terminal_index = n.as<int>();
      }

      if (task_config["input_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, input_remapping is no longer supported use 'remapping'");

      if (task_config["output_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, output_remapping is no longer supported use 'remapping'");

      if (YAML::Node n = task_config["remapping"])
        remapping = n.as<std::map<std::string, std::string>>();

      if (task_config["input_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, input_indexing is no longer supported use 'indexing'");

      if (task_config["output_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, output_indexing is no longer supported use 'indexing'");

      if (YAML::Node n = task_config["indexing"])
        indexing = n.as<std::vector<std::string>>();
      else
        throw std::runtime_error("RasterMotionWithApproachTask, entry 'raster' missing 'indexing' entry");
    }
    else
    {
      throw std::runtime_error("RasterMotionWithApproachTask, entry 'raster' missing 'config' entry");
    }

    if (has_abort_terminal_entry)
    {
      raster_task_factory_ = [task_name, abort_terminal_index, remapping, indexing, &plugin_factory](
                                 const std::string& name, std::size_t index) {
        auto tr = createTask(name, task_name, remapping, indexing, plugin_factory, index);
        static_cast<TaskComposerGraph&>(*tr.node).setTerminalTriggerAbortByIndex(abort_terminal_index);
        return tr;
      };
    }
    else
    {
      raster_task_factory_ = [task_name, remapping, indexing, &plugin_factory](const std::string& name,
                                                                               std::size_t index) {
        return createTask(name, task_name, remapping, indexing, plugin_factory, index);
      };
    }
  }
  else
  {
    throw std::runtime_error("RasterMotionWithApproachTask: missing 'raster' entry");
  }

  if (YAML::Node transition_config = config["transition"])
  {
    std::string task_name;
    bool has_abort_terminal_entry{ false };
    int abort_terminal_index{ -1 };
    std::vector<std::string> indexing;
    std::map<std::string, std::string> remapping;

    if (YAML::Node n = transition_config["task"])
      task_name = n.as<std::string>();
    else
      throw std::runtime_error("RasterMotionWithApproachTask, entry 'transition' missing 'task' entry");

    if (YAML::Node task_config = transition_config["config"])
    {
      if (YAML::Node n = task_config["abort_terminal"])
      {
        has_abort_terminal_entry = true;
        abort_terminal_index = n.as<int>();
      }

      if (task_config["input_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, input_remapping is no longer supported use 'remapping'");

      if (task_config["output_remapping"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, output_remapping is no longer supported use 'remapping'");

      if (YAML::Node n = task_config["remapping"])
        remapping = n.as<std::map<std::string, std::string>>();

      if (task_config["input_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, input_indexing is no longer supported use 'indexing'");

      if (task_config["output_indexing"])  // NOLINT
        throw std::runtime_error("RasterMotionWithApproachTask, output_indexing is no longer supported use 'indexing'");

      if (YAML::Node n = task_config["indexing"])
        indexing = n.as<std::vector<std::string>>();
      else
        throw std::runtime_error("RasterMotionWithApproachTask, entry 'transition' missing 'indexing' entry");
    }
    else
    {
      throw std::runtime_error("RasterMotionWithApproachTask, entry 'transition' missing 'config' entry");
    }

    if (has_abort_terminal_entry)
    {
      transition_task_factory_ = [task_name, abort_terminal_index, remapping, indexing, &plugin_factory](
                                     const std::string& name, std::size_t index) {
        auto tr = createTask(name, task_name, remapping, indexing, plugin_factory, index);
        static_cast<TaskComposerGraph&>(*tr.node).setTerminalTriggerAbortByIndex(abort_terminal_index);
        return tr;
      };
    }
    else
    {
      transition_task_factory_ = [task_name, remapping, indexing, &plugin_factory](const std::string& name,
                                                                                   std::size_t index) {
        return createTask(name, task_name, remapping, indexing, plugin_factory, index);
      };
    }
  }
  else
  {
    throw std::runtime_error("RasterMotionWithApproachTask: missing 'transition' entry");
  }
}

bool RasterMotionWithApproachTask::operator==(const RasterMotionWithApproachTask& rhs) const { return (TaskComposerTask::operator==(rhs)); }
bool RasterMotionWithApproachTask::operator!=(const RasterMotionWithApproachTask& rhs) const { return !operator==(rhs); }

template <class Archive>
void RasterMotionWithApproachTask::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(TaskComposerTask);
}

std::unique_ptr<TaskComposerNodeInfo> RasterMotionWithApproachTask::runImpl(TaskComposerContext& context,
                                                                OptionalTaskComposerExecutor executor) const
{
  // Get the problem
  auto& problem = dynamic_cast<PlanningTaskComposerProblem&>(*context.problem);

  auto info = std::make_unique<MotionPlannerTaskInfo>(*this);
  info->return_value = 0;
  info->status_code = 0;
  info->env = problem.env;

  // --------------------
  // Check that inputs are valid
  // --------------------
  auto input_data_poly = context.data_storage->getData(input_keys_[0]);
  try
  {
    checkTaskInput(input_data_poly);
  }
  catch (const std::exception& e)
  {
    info->status_message = e.what();
    CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
    return info;
  }

  auto& program = input_data_poly.template as<CompositeInstruction>();
  TaskComposerGraph task_graph;

  tesseract_common::ManipulatorInfo program_manip_info = program.getManipulatorInfo().getCombined(problem.manip_info);

  // Start Task
  auto start_task = std::make_unique<StartTask>();
  auto start_uuid = task_graph.addNode(std::move(start_task));

  // Vectors to store task information
  std::vector<std::pair<boost::uuids::uuid, std::pair<std::string, std::string>>> approach_tasks;
  std::vector<std::pair<boost::uuids::uuid, std::pair<std::string, std::string>>> raster_tasks;
  std::vector<std::pair<boost::uuids::uuid, std::pair<std::string, std::string>>> retraction_tasks;
  std::vector<std::pair<boost::uuids::uuid, std::pair<std::string, std::string>>> transition_tasks;

  // Process from_start
  auto from_start = program[0].template as<CompositeInstruction>();
  from_start.setManipulatorInfo(from_start.getManipulatorInfo().getCombined(program_manip_info));
  
  // Count the number of raster_with_approach segments (excluding from_start, transitions, and to_end)
  std::size_t num_raster_with_approach = 0;
  for (std::size_t i = 1; i < program.size() - 1; ++i)
  {
    if (program[i].getDescription().find("Transition") == std::string::npos)
      num_raster_with_approach++;
  }
  
  approach_tasks.reserve(num_raster_with_approach);
  raster_tasks.reserve(num_raster_with_approach);
  retraction_tasks.reserve(num_raster_with_approach);
  
  // Process each raster_with_approach segment
  std::size_t raster_idx = 0;
  for (std::size_t i = 1; i < program.size() - 1; ++i)
  {
    // Skip transition segments
    if (program[i].getDescription().find("Transition") != std::string::npos)
    {
      // Process transition segment
      auto transition_input = program[i].template as<CompositeInstruction>();
      transition_input.setManipulatorInfo(transition_input.getManipulatorInfo().getCombined(program_manip_info));
      
      const std::string transition_task_name = transition_input.getDescription();
      auto transition_results = transition_task_factory_(transition_task_name, transition_tasks.size() + 1);
      transition_results.node->setConditional(false);
      auto transition_uuid = task_graph.addNode(std::move(transition_results.node));
      transition_tasks.emplace_back(transition_uuid, std::make_pair(transition_results.input_key, transition_results.output_key));
      context.data_storage->setData(transition_results.input_key, transition_input);
      
      continue;
    }
    
    // Process raster_with_approach segment
    auto raster_with_approach = program[i].template as<CompositeInstruction>();
    
    // Extract approach, raster, and retraction segments
    auto approach_input = raster_with_approach[0].template as<CompositeInstruction>();
    auto raster_input = raster_with_approach[1].template as<CompositeInstruction>();
    auto retraction_input = raster_with_approach[2].template as<CompositeInstruction>();
    
    // Set manipulator info
    approach_input.setManipulatorInfo(approach_input.getManipulatorInfo().getCombined(program_manip_info));
    raster_input.setManipulatorInfo(raster_input.getManipulatorInfo().getCombined(program_manip_info));
    retraction_input.setManipulatorInfo(retraction_input.getManipulatorInfo().getCombined(program_manip_info));
    
    // Process approach segment
    const std::string approach_task_name = approach_input.getDescription();
    auto approach_results = freespace_task_factory_(approach_task_name, raster_idx * 3 + 1);
    approach_results.node->setConditional(false);
    auto approach_uuid = task_graph.addNode(std::move(approach_results.node));
    approach_tasks.emplace_back(approach_uuid, std::make_pair(approach_results.input_key, approach_results.output_key));
    context.data_storage->setData(approach_results.input_key, approach_input);
    
    // Add edge from start to first approach, or from previous transition to approach
    if (raster_idx == 0)
    {
      task_graph.addEdges(start_uuid, { approach_uuid });
    }
    else if (!transition_tasks.empty())
    {
      const auto& prev_transition = transition_tasks.back();
      const auto& prev_transition_output_key = prev_transition.second.second;
      
      auto update_approach_start_task = std::make_unique<UpdateStartStateTask>(
          "UpdateApproachStartStateTask", approach_results.input_key, prev_transition_output_key, approach_results.output_key, false);
      auto update_approach_start_uuid = task_graph.addNode(std::move(update_approach_start_task));
      
      task_graph.addEdges(update_approach_start_uuid, { approach_uuid });
      task_graph.addEdges(prev_transition.first, { update_approach_start_uuid });
    }
    
    // Process raster segment
    // Get the last instruction from approach to use as the first instruction in raster
    const auto* approach_li = approach_input.getLastMoveInstruction();
    assert(approach_li != nullptr);
    raster_input.insertMoveInstruction(raster_input.begin(), *approach_li);
    
    const std::string raster_task_name = raster_input.getDescription();
    auto raster_results = raster_task_factory_(raster_task_name, raster_idx * 3 + 2);
    raster_results.node->setConditional(false);
    auto raster_uuid = task_graph.addNode(std::move(raster_results.node));
    raster_tasks.emplace_back(raster_uuid, std::make_pair(raster_results.input_key, raster_results.output_key));
    context.data_storage->setData(raster_results.input_key, raster_input);
    
    // Connect approach to raster
    const auto& approach_output_key = approach_tasks.back().second.second;
    auto update_raster_start_task = std::make_unique<UpdateStartStateTask>(
        "UpdateRasterStartStateTask", raster_results.input_key, approach_output_key, raster_results.output_key, false);
    auto update_raster_start_uuid = task_graph.addNode(std::move(update_raster_start_task));
    
    task_graph.addEdges(update_raster_start_uuid, { raster_uuid });
    task_graph.addEdges(approach_tasks.back().first, { update_raster_start_uuid });
    
    // Process retraction segment
    // Get the last instruction from raster to use as the first instruction in retraction
    const auto* raster_li = raster_input.getLastMoveInstruction();
    assert(raster_li != nullptr);
    retraction_input.insertMoveInstruction(retraction_input.begin(), *raster_li);
    
    const std::string retraction_task_name = retraction_input.getDescription();
    auto retraction_results = freespace_task_factory_(retraction_task_name, raster_idx * 3 + 3);
    retraction_results.node->setConditional(false);
    auto retraction_uuid = task_graph.addNode(std::move(retraction_results.node));
    retraction_tasks.emplace_back(retraction_uuid, std::make_pair(retraction_results.input_key, retraction_results.output_key));
    context.data_storage->setData(retraction_results.input_key, retraction_input);
    
    // Connect raster to retraction
    const auto& raster_output_key = raster_tasks.back().second.second;
    auto update_retraction_start_task = std::make_unique<UpdateStartStateTask>(
        "UpdateRetractionStartStateTask", retraction_results.input_key, raster_output_key, retraction_results.output_key, false);
    auto update_retraction_start_uuid = task_graph.addNode(std::move(update_retraction_start_task));
    
    task_graph.addEdges(update_retraction_start_uuid, { retraction_uuid });
    task_graph.addEdges(raster_tasks.back().first, { update_retraction_start_uuid });
    
    raster_idx++;
  }
  
  // Process to_end
  auto to_end = program.back().template as<CompositeInstruction>();
  to_end.setManipulatorInfo(to_end.getManipulatorInfo().getCombined(program_manip_info));
  
  const std::string to_end_task_name = to_end.getDescription();
  auto to_end_results = freespace_task_factory_(to_end_task_name, program.size());
  auto to_end_uuid = task_graph.addNode(std::move(to_end_results.node));
  context.data_storage->setData(to_end_results.input_key, to_end);
  
  // Connect last retraction to to_end
  if (!retraction_tasks.empty())
  {
    const auto& last_retraction = retraction_tasks.back();
    const auto& last_retraction_output_key = last_retraction.second.second;
    
    auto update_to_end_start_task = std::make_unique<UpdateStartStateTask>(
        "UpdateToEndStartStateTask", to_end_results.input_key, last_retraction_output_key, to_end_results.output_key, false);
    auto update_to_end_start_uuid = task_graph.addNode(std::move(update_to_end_start_task));
    
    task_graph.addEdges(update_to_end_start_uuid, { to_end_uuid });
    task_graph.addEdges(last_retraction.first, { update_to_end_start_uuid });
  }

  TaskComposerFuture::UPtr future = executor.value().get().run(task_graph, context.problem, context.data_storage);
  future->wait();

  // Merge child context data into parent context
  context.task_infos.mergeInfoMap(std::move(future->context->task_infos));
  if (future->context->isAborted())
    context.abort(future->context->task_infos.getAbortingNode());

  auto info_map = context.task_infos.getInfoMap();
  if (context.problem->dotgraph)
  {
    std::stringstream dot_graph;
    dot_graph << "subgraph cluster_" << toString(uuid_) << " {\n color=black;\n label = \"" << name_ << "\\n("
              << uuid_str_ << ")\";\n";
    task_graph.dump(dot_graph, this, info_map);  // dump the graph including dynamic tasks
    dot_graph << "}\n";
    info->dotgraph = dot_graph.str();
  }

  if (context.isAborted())
  {
    info->status_message = "Raster with approach subgraph failed";
    CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
    return info;
  }

  // Rebuild the program with the results
  program.clear();
  
  // Add from_start
  program.push_back(from_start);
  
  // Add approach/raster/retraction segments and transitions
  for (std::size_t i = 0; i < approach_tasks.size(); ++i)
  {
    // Create a new composite instruction for this set of approach/raster/retraction
    CompositeInstruction raster_with_approach(program.getProfile(), tesseract_planning::CompositeInstructionOrder::ORDERED, program.getManipulatorInfo());
    
    // Add approach
    const auto& approach_output_key = approach_tasks[i].second.second;
    CompositeInstruction approach = context.data_storage->getData(approach_output_key).as<CompositeInstruction>();
    raster_with_approach.push_back(approach);
    
    // Add raster
    const auto& raster_output_key = raster_tasks[i].second.second;
    CompositeInstruction raster = context.data_storage->getData(raster_output_key).as<CompositeInstruction>();
    raster.erase(raster.begin()); // Remove the first instruction which was copied from approach
    raster_with_approach.push_back(raster);
    
    // Add retraction
    const auto& retraction_output_key = retraction_tasks[i].second.second;
    CompositeInstruction retraction = context.data_storage->getData(retraction_output_key).as<CompositeInstruction>();
    retraction.erase(retraction.begin()); // Remove the first instruction which was copied from raster
    raster_with_approach.push_back(retraction);
    
    // Add the raster_with_approach composite to the program
    program.push_back(raster_with_approach);
    
    // Add transition if not the last set
    if (i < approach_tasks.size() - 1 && i < transition_tasks.size())
    {
      const auto& transition_output_key = transition_tasks[i].second.second;
      CompositeInstruction transition = context.data_storage->getData(transition_output_key).as<CompositeInstruction>();
      program.push_back(transition);
    }
  }
  
  // Add to_end
  program.push_back(to_end);

  context.data_storage->setData(output_keys_[0], program);

  info->color = "green";
  info->status_code = 1;
  info->status_message = "Successful";
  info->return_value = 1;
  return info;
}

void RasterMotionWithApproachTask::checkTaskInput(const tesseract_common::AnyPoly& input)
{
  // -------------
  // Check Input
  // -------------
  if (input.isNull())
    throw std::runtime_error("RasterMotionWithApproachTask, input is null");

  if (input.getType() != std::type_index(typeid(CompositeInstruction)))
    throw std::runtime_error("RasterMotionWithApproachTask, input is not a composite instruction");

  const auto& composite = input.as<CompositeInstruction>();

  // Check that we have at least from_start, one raster_with_approach, and to_end
  if (composite.size() < 3)
    throw std::runtime_error("RasterMotionWithApproachTask, input must contain at least from_start, one raster_with_approach, and to_end");

  // Check all instructions are composites
  // for (std::size_t index = 0; index < composite.size(); index++)
  // {
  //   if (!composite.at(index).isCompositeInstruction())
  //     throw std::runtime_error("RasterMotionWithApproachTask, all instructions should be composites");
  // }

  // Check that each raster_with_approach composite contains approach, raster, and retraction
  for (std::size_t index = 1; index < composite.size() - 1; index++)
  {
    // Skip transition composites
    if (composite.at(index).getDescription().find("Transition") != std::string::npos)
      continue;

    const auto& raster_with_approach = composite.at(index).as<CompositeInstruction>();
    
    // Check that raster_with_approach has 3 children (approach, raster, retraction)
    if (raster_with_approach.size() != 3)
      throw std::runtime_error("RasterMotionWithApproachTask, each raster_with_approach must contain exactly 3 children (approach, raster, retraction)");
    
    // Check that all children are composites
    for (std::size_t child_index = 0; child_index < raster_with_approach.size(); child_index++)
    {
      if (!raster_with_approach.at(child_index).isCompositeInstruction())
        throw std::runtime_error("RasterMotionWithApproachTask, all children of raster_with_approach should be composites");
    }
  }
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RasterMotionWithApproachTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RasterMotionWithApproachTask)
