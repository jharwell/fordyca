/**
 * @file memory_foraging_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_MEMORY_FORAGING_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_MEMORY_FORAGING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include <boost/shared_ptr.hpp>
#include "fordyca/controller/random_foraging_controller.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/fsm/memory_foraging_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace visitor = rcppsw::patterns::visitor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief  A controller is simply an implementation of the CCI_Controller class.
 */
class memory_foraging_controller : public random_foraging_controller,
                                   public visitor::visitable<memory_foraging_controller> {
 public:
  memory_foraging_controller(void) :
      random_foraging_controller(),
      m_display_los(false),
      m_light_loc(),
      m_map(),
      m_fsm() {}

  void display_los(bool display_los) { m_display_los = display_los; }

  /**
   * @brief If TRUE, then the robot should display its approxibate LOS as a
   * circle on the ground during simulation.
   */
  bool display_los(void) const { return m_display_los; }

  /**
   * @brief If TRUE, the robot is currently searching for a block.
   */
  bool is_exploring(void) const { return m_fsm->is_exploring(); }

  /**
   * @brief If TRUE, the robot is current engaged in collision avoidance.
   */
  bool is_avoiding_collision(void) const { return m_fsm->is_avoiding_collision(); }

  /**
   * @brief If TRUE, the robot is currently returning to the nest carrying a block.
   */
  bool is_returning(void) const { return m_fsm->is_returning(); }

  /**
   * @brief If TRUE, then the robot is currently searching for a block.
   */
  bool is_searching_for_block(void) const { return m_fsm->is_searching_for_block(); }

  bool is_vectoring(void) const { return m_fsm->is_vectoring(); }

  /*
   * @brief Initialize the controller.
   *
   * @param t_node Points to the <parameters> section in the XML file in the
   *               <controllers><memory_foraging_controller_controller> section.
   */
  void Init(argos::TConfigurationNode& t_node) override;

  /*
   * @brief Called once every time step; length set in the XML file.
   *
   * Since the FSM does most of the work, this function just tells it run.
   */
  void ControlStep(void) override;

  /**
   * @brief Set the robot's current line of sight (LOS). This sort of a hack,
   * but is much easier than actually computing it, and helps me get on with teh
   * actual reserach I'm interested in.
   */
  void los(std::unique_ptr<representation::line_of_sight>& new_los) {
    sensors()->los(new_los);
  }

  /**
   * @brief Get the current LOS for the robot.
   *
   * @return The current LOS.
   */
  const representation::line_of_sight* los(void) const { return sensors()->los(); }

  /**
   * @brief Set the current clock tick. In a real world, each robot would
   * maintain its own clock tick, and overall there would no doubt be
   * considerable skew; this is a simulation hack that makes things much
   * nicer/easier to deal with.
   */
  void tick(uint tick) { sensors()->tick(tick); }

  /**
   * @brief Set the current location of the robot.
   *
   * This is a hack, as real world robot's would have to do their own
   * localization. This is far superior to that, in terms of ease of
   * programming. Plus it helps me focus in on my actual research. Ideally,
   * robot's would calculate this from sensor values, rather than it being set
   * by the loop functions.
   */
  void robot_loc(argos::CVector2 loc) { return sensors()->robot_loc(loc); }
  argos::CVector2 robot_loc(void) { return sensors()->robot_loc(); }

  representation::perceived_arena_map* map(void) const { return m_map.get(); }

 private:
  /** Should the robot's LOS be displayed as a circle?  */
  bool                                                 m_display_los;
  argos::CVector2                                      m_light_loc;
  std::shared_ptr<representation::perceived_arena_map> m_map;
  std::shared_ptr<fsm::memory_foraging_fsm>            m_fsm;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_MEMORY_FORAGING_CONTROLLER_CONTROLLER_HPP_ */
