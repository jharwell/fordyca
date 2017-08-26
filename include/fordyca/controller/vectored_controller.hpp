/**
 * @file vectored_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_VECTORED_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_VECTORED_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include <boost/shared_ptr.hpp>
#include "fordyca/controller/base_controller.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief  A controller is simply an implementation of the CCI_Controller class.
 */
class vectored_controller : public base_controller {
 public:
  vectored_controller(void) :
      base_controller(),
      m_display_los(false),
      m_light_loc(),
      m_map() {}

  void display_los(bool display_los) { m_display_los = display_los; }
  bool display_los(void) const { return m_display_los; }

  /*
   * @brief Initialize the controller.
   *
   * @param t_node Points to the <parameters> section in the XML file in the
   *               <controllers><vectored_controller_controller> section.
   */
  virtual void Init(argos::TConfigurationNode& t_node);

  /*
   * @brief Called once every time step; length set in the XML file.
   *
   * Since the FSM does most of the work, this function just tells it run.
   */
  virtual void ControlStep(void);

  void los(std::unique_ptr<representation::line_of_sight>& new_los) {
    sensors()->los(new_los);
  }
  const representation::line_of_sight* los(void) const { return sensors()->los(); }

  representation::discrete_coord robot_loc(void) { return sensors()->los()->center(); }

  /**
   * @brief Pickup a block the robot is currently on top of, updating state as appropriate.
   *
   * This needs to be here, rather than in the FSM, because picking up blocks
   * needs to be handled in the loop functions so the area can correctly be drawn
   * each timestep.
   */
  virtual void pickup_block(representation::block* block);

 private:
  bool                                                 m_display_los;
  argos::CVector2                                      m_light_loc;
  std::unique_ptr<representation::perceived_arena_map> m_map;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_VECTORED_CONTROLLER_CONTROLLER_HPP_ */
