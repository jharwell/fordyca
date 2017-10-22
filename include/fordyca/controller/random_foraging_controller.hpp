/**
 * @file random_foraging_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_RANDOM_FORAGING_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_RANDOM_FORAGING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/rng.h>
#include <boost/shared_ptr.hpp>
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/fsm/random_foraging_fsm.hpp"
#include "fordyca/controller/sensor_manager.hpp"
#include "fordyca/controller/actuator_manager.hpp"
#include "fordyca/controller/base_foraging_controller.hpp"
#include "fordyca/controller/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class random_foraging_controller
 *
 * @brief The most basic form of a foraging controller: roam around randomly
 * until you find a block, and then bring it back to the nest; repeat.
 */
class random_foraging_controller : public base_foraging_controller,
                                   public visitor::visitable<random_foraging_controller> {
 public:
  random_foraging_controller(void);

  /**
   * @brief If TRUE, the robot is currently searching for a block.
   */
  bool is_exploring(void) const { return m_fsm->is_exploring(); }

  /**
   * @brief If \c TRUE, the robot is currently returning to the nest carrying a block.
   */
  bool is_transporting_to_nest(void) const { return m_fsm->is_transporting_to_nest(); }

  /**p
   * @brief If \c TRUE, the robot is current engaged in collision avoidance.
   */
  bool is_avoiding_collision(void) const { return m_fsm->is_avoiding_collision(); }

  /*
   * @brief Initialize the controller.
   *
   * @param t_node Points to the <parameters> section in the XML file in the
   *               <controllers><random_foraging_controller_controller> section.
   */
  void Init(argos::TConfigurationNode& t_node) override;

  /*
   * @brief Called once every time step; length set in the XML file.
   *
   * Since the FSM does all the work, this function just tells it run.
   */
  void ControlStep(void) override { m_fsm->run(); }

  /*
   * @brief Reset controller to its state right after the Init().
   */
  void Reset(void) override;

  fsm::random_foraging_fsm* fsm(void) const { return m_fsm.get(); }

 private:
  std::unique_ptr<fsm::random_foraging_fsm>  m_fsm;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_RANDOM_FORAGING_CONTROLLER_CONTROLLER_HPP_ */
