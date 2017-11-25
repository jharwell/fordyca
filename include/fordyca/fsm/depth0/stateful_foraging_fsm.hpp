/**
 * @file stateful_foraging_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH0_STATEFUL_FORAGING_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH0_STATEFUL_FORAGING_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/task_allocation/taskable.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/stateless_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/depth0_metrics.hpp"

#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/block_to_nest_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params {
struct fsm_params;
} /* namespace params */

namespace controller {
namespace depth0 { class foraging_sensors; }
namespace depth1 { class foraging_sensors; }
class actuator_manager;
} /* namespace controller */

namespace representation {
class perceived_arena_map;
} /* namespace representation */

namespace visitor = rcppsw::patterns::visitor;
namespace task_allocation = rcppsw::task_allocation;

NS_START(fsm, depth0);

namespace rmetrics = metrics::collectible_metrics::robot_metrics;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief The FSM for an unpartitioned foraging task. Each robot executing this
 * FSM will locate for a block (either a known block or via random exploration),
 * pickup the block and bring it all the way back to the nest.
 */
class stateful_foraging_fsm : public base_foraging_fsm,
                     public rmetrics::stateless_metrics,
                     public rmetrics::depth0_metrics,
                     public task_allocation::taskable,
                     public visitor::visitable_any<depth0::stateful_foraging_fsm> {
 public:
  stateful_foraging_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::depth1::foraging_sensors>& sensors,
      const std::shared_ptr<controller::actuator_manager>& actuators,
      const std::shared_ptr<const representation::perceived_arena_map>& map);

  /* taskable overrides */

  /**
   * @brief Reset the memory foraging task to a state where it can be restarted.
   */
  void task_reset(void) override { init(); }

  void task_start(__unused const task_allocation::taskable_argument* const arg) override {}


  /**
   * @brief Run the memory foraging task.
   */
  void task_execute(void) override;

  /**
   * @brief Determine if a block has been retrieved, brought to the nest, and
   * the robot has left the nest, ready for its next task.
   *
   * @return \c TRUE if the condition is met, \c FALSE otherwise.
   */
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }

  bool task_running(void) const override { return m_task_running; }

  /* base metrics */
  bool is_exploring_for_block(void) const override;
  bool is_avoiding_collision(void) const override;
  bool is_transporting_to_nest(void) const override;

  /* depth0 metrics */
  bool is_acquiring_block(void) const override;
  bool is_vectoring_to_block(void) const override;

  bool block_acquired(void) const;

  /**
   * @brief Reset the FSM.
   */
  void init(void) override;

  controller::depth0::foraging_sensors* sensors(void) const { return m_sensors.get(); }

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_FREE_BLOCK,    /* superstate for finding a block */
    ST_LEAVING_NEST,          /* Block dropped in nest--time to go */
    ST_FINISHED,
    ST_MAX_STATES
  };

 private:
  /* inherited states */
  HFSM_STATE_INHERIT(base_foraging_fsm, leaving_nest,
                     state_machine::event_data);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_leaving_nest);

  /* depth0 foraging states */
  HFSM_STATE_DECLARE(stateful_foraging_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE(stateful_foraging_fsm, block_to_nest, state_machine::event_data);
  HFSM_STATE_DECLARE_ND(stateful_foraging_fsm, finished);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return &mc_state_map[index];
  }

  stateful_foraging_fsm(const stateful_foraging_fsm& fsm) = delete;
  stateful_foraging_fsm& operator=(const stateful_foraging_fsm& fsm) = delete;

  /* data members */
  bool m_task_running;
  std::shared_ptr<controller::depth0::foraging_sensors>  m_sensors;
  block_to_nest_fsm m_block_fsm;
  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(depth0, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH0_STATEFUL_FORAGING_FSM_HPP_ */
