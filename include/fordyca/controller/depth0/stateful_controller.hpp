/**
 * @file stateful_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH0_STATEFUL_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH0_STATEFUL_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/tasks/base_foraging_task.hpp"
#include "fordyca/metrics/world_model_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace fsm { namespace depth0 { class stateful_fsm; }}
NS_START(controller);
class base_perception_subsystem;
class block_sel_matrix;
namespace depth0 { class sensing_subsystem; }

NS_START(depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class stateful_controller
 * @ingroup controller depth0
 *
 * @brief A foraging controller that remembers what it has seen for a period of
 * time (knowledge decays according to an exponential model,
 * @see pheromone_density).
 */
class stateful_controller : public crw_controller,
                            public er::client<stateful_controller>,
                            public metrics::world_model_metrics,
                            public visitor::visitable_any<stateful_controller> {
 public:
  stateful_controller(void);
  ~stateful_controller(void) override;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;
  void Reset(void) override;

  /* goal acquisition metrics */
  FSM_WRAPPER_DECLAREC(bool, goal_acquired);
  FSM_WRAPPER_DECLAREC(acquisition_goal_type, acquisition_goal);

  /* world model metrics */
  uint cell_state_inaccuracies(uint state) const override;
  double known_percentage(void) const override;
  double unknown_percentage(void) const override;

  /* block transportation */
  FSM_WRAPPER_DECLAREC(transport_goal_type, block_transport_goal);

  /**
   * @brief Set the robot's current line of sight (LOS).
   */
  void los(std::unique_ptr<representation::line_of_sight>& new_los);
  double los_dim(void) const;


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

  const fsm::depth0::stateful_fsm* fsm(void) const { return m_fsm.get(); }
  fsm::depth0::stateful_fsm* fsm(void) { return m_fsm.get(); }

  const class block_sel_matrix* block_sel_matrix(void) const {
    return m_block_sel_matrix.get();
  }
  /*
   * Needed to update block selections exceptions list in depth2.
   */
  class block_sel_matrix* block_sel_matrix(void) {
    return m_block_sel_matrix.get();
  }

 protected:
  void perception(std::unique_ptr<base_perception_subsystem> perception);
  void block_sel_matrix(std::unique_ptr<class block_sel_matrix> m);

 private:
  // clang-format off
  bool                                       m_display_los{false};
  rmath::vector2d                            m_light_loc;
  std::unique_ptr<class block_sel_matrix>    m_block_sel_matrix;
  std::unique_ptr<base_perception_subsystem> m_perception;
  std::unique_ptr<fsm::depth0::stateful_fsm> m_fsm;
  // clang-format on
};

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_STATEFUL_CONTROLLER_HPP_ */
