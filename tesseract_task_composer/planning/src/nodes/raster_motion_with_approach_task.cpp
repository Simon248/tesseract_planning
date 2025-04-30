/**
 * @file raster_motion_with_approach_task.cpp
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
/////  DEBUG
#include <sstream>
#include <fstream>
#include <iostream>
#include <filesystem> 


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
                                   TaskFactory approach_and_retreat_task_factory,
                                   TaskFactory raster_task_factory)
  : TaskComposerTask(std::move(name), conditional)
  , approach_and_retreat_task_factory_(std::move(approach_and_retreat_task_factory))
  , raster_task_factory_(std::move(raster_task_factory))
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

  if (YAML::Node ApproachAndRetreat_config = config["ApproachAndRetreat"])
  {
    std::string task_name;
    bool has_abort_terminal_entry{ false };
    int abort_terminal_index{ -1 };
    std::vector<std::string> indexing;
    std::map<std::string, std::string> remapping;

    if (YAML::Node n = ApproachAndRetreat_config["task"])
      task_name = n.as<std::string>();
    else
      throw std::runtime_error("RasterMotionWithApproachTask, entry 'freespace' missing 'task' entry");

    if (YAML::Node task_config = ApproachAndRetreat_config["config"])
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
        throw std::runtime_error("RasterMotionWithApproachTask, entry 'ApproachAndRetreat' missing 'indexing' entry");
    }
    else
    {
      throw std::runtime_error("RasterMotionWithApproachTask, entry 'ApproachAndRetreat' missing 'config' entry");
    }

    if (has_abort_terminal_entry)
    {
      approach_and_retreat_task_factory_ = [task_name, abort_terminal_index, remapping, indexing, &plugin_factory](
                                    const std::string& name, std::size_t index) {
        auto tr = createTask(name, task_name, remapping, indexing, plugin_factory, index);
        static_cast<TaskComposerGraph&>(*tr.node).setTerminalTriggerAbortByIndex(abort_terminal_index);
        return tr;
      };
    }
    else
    {
      approach_and_retreat_task_factory_ = [task_name, remapping, indexing, &plugin_factory](const std::string& name,
                                                                                  std::size_t index) {
        return createTask(name, task_name, remapping, indexing, plugin_factory, index);
      };
    }
  }
  else
  {
    throw std::runtime_error("RasterMotionWithApproachTask: missing 'ApproachAndRetreat' entry");
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

  auto& raster_with_approach = input_data_poly.template as<CompositeInstruction>();
  TaskComposerGraph task_graph;

//   //////////////DEBUG/////////////////////

// // Sauvegarder le buffer original de cout

// std::stringstream buffer;
// std::streambuf* cout_buf = std::cout.rdbuf(); // sauvegarde cout

// std::cout.rdbuf(buffer.rdbuf());              // redirige cout vers buffer
// raster_with_approach.print("Program: ");
// std::cout.rdbuf(cout_buf);                    // restaure cout

// // Maintenant le contenu est dans buffer
// std::ofstream out("/tmp/tess_dot_graph/program_raster_with_approach_task_log.txt", std::ios::app);
// if (!out)
// {
//   std::cerr << "Erreur : impossible d'ouvrir le fichier" << std::endl;
// }
// else
// {
//   out << buffer.str(); // écrit seulement ce que tu veux
// }
// //////////////////////////////////




  tesseract_common::ManipulatorInfo program_manip_info = raster_with_approach.getManipulatorInfo().getCombined(problem.manip_info);

  // Start Task
  auto start_task = std::make_unique<StartTask>();
  auto start_uuid = task_graph.addNode(std::move(start_task));

  // Extract the approach, raster, and retraction segments
  auto approach_input = raster_with_approach[0].template as<CompositeInstruction>();
  auto raster_input = raster_with_approach[1].template as<CompositeInstruction>();
  auto retraction_input = raster_with_approach[2].template as<CompositeInstruction>();

  // Set the manipulator info
  approach_input.setManipulatorInfo(approach_input.getManipulatorInfo().getCombined(program_manip_info));
  raster_input.setManipulatorInfo(raster_input.getManipulatorInfo().getCombined(program_manip_info));
  retraction_input.setManipulatorInfo(retraction_input.getManipulatorInfo().getCombined(program_manip_info));

  // Process the raster segment first
  const std::string raster_task_name = "Raster: " + raster_input.getDescription();


  // Extract the instance index from the parent task name
  // The parent task name is in the format "Raster #X: ..."
  // We can extract this from the input_keys_[0] which is in the format "RasterWithApproachPipeline_output_dataX"
  std::size_t instance_idx = 0;
  std::string input_key = input_keys_[0];
  size_t data_pos = input_key.find("_data");
  if (data_pos != std::string::npos) {
    std::string idx_str = input_key.substr(data_pos + 5);
    try {
      instance_idx = std::stoull(idx_str); 
    } catch (...) {
      instance_idx = 0;
    }
  }

  // Use instance_idx to create unique indices for subtasks
  // Base index for this instance (each instance gets 3 indices)
  std::size_t base_idx = (instance_idx * 3) + 1;
  auto raster_results = raster_task_factory_(raster_task_name, base_idx);
  raster_results.node->setConditional(false);
  auto raster_uuid = task_graph.addNode(std::move(raster_results.node));
  context.data_storage->setData(raster_results.input_key, raster_input);

  // Process the approach segment
  // Use the first point of the raster for the approach
  const auto* first_raster_point = raster_input.getFirstMoveInstruction();
  assert(first_raster_point != nullptr);
  approach_input.insertMoveInstruction(approach_input.end(), *first_raster_point);

  const std::string approach_task_name = "Approach: " + approach_input.getDescription();
  auto approach_results = approach_and_retreat_task_factory_(approach_task_name, base_idx + 1);
  approach_results.node->setConditional(false);
  auto approach_uuid = task_graph.addNode(std::move(approach_results.node));
  context.data_storage->setData(approach_results.input_key, approach_input);

  // Process the retraction segment
  // Use the last point of the raster for the retraction
  const auto* last_raster_point = raster_input.getLastMoveInstruction();
  assert(last_raster_point != nullptr);
  retraction_input.insertMoveInstruction(retraction_input.begin(), *last_raster_point);

  const std::string retraction_task_name = "Retraction: " + retraction_input.getDescription();
  auto retraction_results = approach_and_retreat_task_factory_(retraction_task_name, base_idx + 2);
  retraction_results.node->setConditional(false);
  auto retraction_uuid = task_graph.addNode(std::move(retraction_results.node));
  context.data_storage->setData(retraction_results.input_key, retraction_input);

  // update start retraction task
  std::string raster_output_key = raster_results.output_key;
  auto update_retraction_start_task = std::make_unique<UpdateStartStateTask>(
      "UpdateRetractionStartStateTask", retraction_results.input_key, raster_output_key, retraction_results.output_key, false);
  auto update_retraction_start_uuid = task_graph.addNode(std::move(update_retraction_start_task));

  // Update the end state of the approach task
  auto update_approach_end_task = std::make_unique<UpdateEndStateTask>(
    "UpdateApproachEndStateTask", approach_results.input_key, raster_output_key, approach_results.output_key, false);
  auto update_approach_end_uuid = task_graph.addNode(std::move(update_approach_end_task));
  

  // Connect the nodes
  task_graph.addEdges(start_uuid, { raster_uuid });
  task_graph.addEdges(raster_uuid, { update_retraction_start_uuid });
  task_graph.addEdges(raster_uuid, { update_approach_end_uuid });
  task_graph.addEdges(update_retraction_start_uuid, { retraction_uuid });
  task_graph.addEdges(update_approach_end_uuid, { approach_uuid });



  // Execute the task graph
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

  // Reconstruct the program
  raster_with_approach.clear();

  // Add the approach segment
  std::string approach_output_key = approach_results.output_key;
  CompositeInstruction approach = context.data_storage->getData(approach_output_key).as<CompositeInstruction>();
  approach.erase(approach.end()); // Remove the last point that was copied from the raster
  raster_with_approach.push_back(approach);

  // Add the raster segment
  CompositeInstruction raster = context.data_storage->getData(raster_output_key).as<CompositeInstruction>();
  raster_with_approach.push_back(raster);

  // Add the retraction segment
  std::string retraction_output_key = retraction_results.output_key;
  CompositeInstruction retraction = context.data_storage->getData(retraction_output_key).as<CompositeInstruction>();
  retraction.erase(retraction.begin()); // Remove the first point that was copied from the raster
  raster_with_approach.push_back(retraction);

  // Store the result
  context.data_storage->setData(output_keys_[0], raster_with_approach);

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
  composite.print(); //contient app1, raster1, retract1, app2, raster2, retract2

  if (composite.size() != 3)
  {
    CONSOLE_BRIDGE_logError("%s", composite);
    throw std::runtime_error("RasterMotionWithApproachTask, input must contain exactly 3 elements (approach, raster, retraction)");
  }

  // Check that the composite has exactly 3 elements (approach, raster, retraction)
  for (std::size_t index = 0; index < composite.size(); index++)
  {
    if (composite.at(index).getType() != std::type_index(typeid(CompositeInstruction)))
    {
    CONSOLE_BRIDGE_logError("%s", composite.at(index));
      throw std::runtime_error("RasterMotionWithApproachTask, input must contain only composite instructions");
    }
  }
  


  // Check that all elements are composites
  for (std::size_t index = 0; index < composite.size(); index++)
  {
    if (!composite.at(index).isCompositeInstruction())
      throw std::runtime_error("RasterMotionWithApproachTask, all elements should be composites");
  }
}

}  // namespace tesseract_planning

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::RasterMotionWithApproachTask)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::RasterMotionWithApproachTask)
