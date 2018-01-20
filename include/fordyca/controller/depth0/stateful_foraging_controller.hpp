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
#include "fordyca/metrics/collectible_metrics/fsm/stateful_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation {
class polled_executive;
class executable_task;
}}

NS_START(fordyca);
namespace tasks { class generalist; class foraging_task; };
namespace representation { class perceived_arena_map; }

NS_START(controller);
namespace visitor = rcppsw::patterns::visitor;
namespace depth1 { class foraging_sensors; }

NS_START(depth0);
namespace rmetrics = metrics::collectible_metrics::fsm;
namespace task_allocation = rcppsw::task_allocation;

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
                                     public rmetrics::stateful_metrics,
                                     public visitor::visitable_any<stateful_foraging_controller> {
 public:
  stateful_foraging_controller(void);

  /* CCI_Controller overrides */
  void Init(argos::TConfigurationNode& node) override;
  void ControlStep(void) override;

  /* stateless metrics */
  bool is_exploring_for_block(void) const override;
  bool is_avoiding_collision(void) const override;
  bool is_transporting_to_nest(void) const override;

  /* stateful metrics */
  bool is_acquiring_block(void) const override;
  bool is_vectoring_to_block(void) const override;

  /* distance metrics */
  size_t entity_id(void) const override;
  double timestep_distance(void) const override;

  bool block_acquired(void) const;

  /**
   * @brief Get the current task the controller is executing. For this
   * controller, that is always the \ref generalist task.
   */
  tasks::foraging_task* current_task(void) const;

  /**
   * @brief Set the robot's current line of sight (LOS).
   */
  void los(std::unique_ptr<representation::line_of_sight>& new_los);

  /**
   * @brief Process the LOS for a given timestep.
   *
   * Only handles blocks within a LOS; caches are ignored.
   */
  virtual void process_los(const representation::line_of_sight* los);

  /**
   * @brief Get the current LOS for the robot.
   */
  const representation::line_of_sight* los(void) const;

  std::shared_ptr<representation::perceived_arena_map>& map_ref(void) {
    return m_map;
  }
  depth1::foraging_sensors* stateful_sensors(void) const;
  std::shared_ptr<depth1::foraging_sensors> stateful_sensors_ref(void) const;

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

  /**
   * @brief Set the current location of the robot.
   *
   * This is a hack, as real world robot's would have to do their own
   * localization. This is far superior to that, in terms of ease of
   * programming. Plus it helps me focus in on my actual research. Ideally,
   * robots would calculate this from sensor values, rather than it being set by
   * the loop functions.
   */
  void robot_loc(argos::CVector2 loc);
  argos::CVector2 robot_loc(void) const;
  representation::perceived_arena_map* map(void) const { return m_map.get(); }

 private:
  // clang-format off
  bool                                                 m_display_los{false};
  argos::CVector2                                      m_light_loc;
  std::shared_ptr<representation::perceived_arena_map> m_map;
  std::unique_ptr<task_allocation::polled_executive>   m_executive;
  std::unique_ptr<tasks::generalist>                   m_generalist;
  // clang-format on
};

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_STATEFUL_FORAGING_CONTROLLER_HPP_ */
