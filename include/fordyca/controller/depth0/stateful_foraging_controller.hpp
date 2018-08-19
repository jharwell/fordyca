/**
 * @file stateful_foraging_controller.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
 *
 * FORDYCA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH0_STATEFUL_FORAGING_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH0_STATEFUL_FORAGING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>

#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/controller/depth0/stateless_foraging_controller.hpp"
#include "fordyca/tasks/base_foraging_task.hpp"
#include "rcppsw/task_allocation/partitionable_task_params.hpp"
#include "fordyca/metrics/world_model_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation {
class bifurcating_tdgraph_executive;
class bifurcating_tab;
class executable_task;
using executive_params = partitionable_task_params;
}}
namespace visitor = rcppsw::patterns::visitor;
namespace ta = rcppsw::task_allocation;

NS_START(fordyca);

namespace tasks { namespace depth0 { class foraging_task; }}

NS_START(controller);

class base_perception_subsystem;
class block_selection_matrix;
namespace depth0 { class sensing_subsystem; }

NS_START(depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class stateful_foraging_controller
 * @ingroup controller depth0
 *
 * @brief A foraging controller that remembers what it has seen for a period of
 * time (knowledge decays according to an exponential model,
 * @see pheromone_density).
 *
 * Robots using this controller execute the \ref generalist task, in which a
 * block is acquired (either via randomized exploring or by vectoring to a known
 * block) and then bring the block to the nest.
 */
class stateful_foraging_controller : public stateless_foraging_controller,
                                     public metrics::world_model_metrics,
                                     public visitor::visitable_any<stateful_foraging_controller> {
 public:
  stateful_foraging_controller(void);
  ~stateful_foraging_controller(void) override;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;
  void Reset(void) override;

  /* goal acquisition metrics */
  FSM_WRAPPER_DECLARE(bool, goal_acquired);
  FSM_WRAPPER_DECLARE(acquisition_goal_type, acquisition_goal);

  /* world model metrics */
  uint cell_state_inaccuracies(uint state) const override;

  /* block transportation */
  FSM_WRAPPER_DECLARE(transport_goal_type, block_transport_goal);

  /**
   * @brief Get the current task the controller is executing. For this
   * controller, that is always the \ref generalist task.
   */
  virtual tasks::base_foraging_task* current_task(void);
  virtual const tasks::base_foraging_task* current_task(void) const;

  /**
   * @brief Set the robot's current line of sight (LOS).
   */
  void los(std::unique_ptr<representation::line_of_sight>& new_los);

  const depth0::sensing_subsystem* stateful_sensors(void) const;
  depth0::sensing_subsystem* stateful_sensors(void);

  /**
   * @brief Get the current LOS for the robot.
   */
  const representation::line_of_sight* los(void) const;

  /**
   * @brief Set whether or not a robot is supposed to display it's LOS as a
   * square of the appropriate size during simulation.
   */
  void display_los(bool display_los) { m_display_los = display_los; }

  /**
   * @brief If \c TRUE, then the robot should display its approximate LOS as a
   * circle on the ground during simulation.
   */
  bool display_los(void) const { return m_display_los; }

  const base_perception_subsystem* perception(void) const { return m_perception.get(); }
  base_perception_subsystem* perception(void) { return m_perception.get(); }

  const ta::bifurcating_tab* active_tab(void) const;

  /*
   * Public to setup metric collection from tasks.
   */
  const ta::bifurcating_tdgraph_executive* executive(void) const { return m_executive.get(); }
  ta::bifurcating_tdgraph_executive* executive(void) { return m_executive.get(); }

 protected:
  void perception(std::unique_ptr<base_perception_subsystem> perception);
  const block_selection_matrix* block_sel_matrix(void) const { return m_block_sel_matrix.get(); }
  void block_sel_matrix(std::unique_ptr<block_selection_matrix> m);

  /*
   * The stateful foraging controller owns the executive, but derived classes
   * can access it and set it to whatever they want. This is done to reduce the
   * amount of function overriding that would have to be performed otherwise if
   * derived controllers each had private executives--slightly cleaner to do it
   * this way I think.
   *
   * Strategy pattern!
   */
  void executive(std::unique_ptr<ta::bifurcating_tdgraph_executive> executive);

 private:
  // clang-format off
  bool                                                 m_display_los{false};
  argos::CVector2                                      m_light_loc;
  std::unique_ptr<block_selection_matrix>              m_block_sel_matrix;
  std::unique_ptr<base_perception_subsystem>           m_perception;
  std::unique_ptr<ta::bifurcating_tdgraph_executive>   m_executive;
  // clang-format on
};

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_STATEFUL_FORAGING_CONTROLLER_HPP_ */
