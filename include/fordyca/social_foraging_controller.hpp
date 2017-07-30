/**
 * @file social_foraging_controllor.hpp
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

#ifndef INCLUDE_FORDYCA_SOCIAL_FORAGING_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_SOCIAL_FORAGING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include <boost/shared_ptr.hpp>
#include "fordyca/fordyca_params.hpp"
#include "fordyca/social_fsm.hpp"
#include "fordyca/parameter_parser.hpp"
#include "fordyca/sensor_manager.hpp"
#include "fordyca/actuator_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief  A controller is simply an implementation of the CCI_Controller class.
 */
class social_foraging_controller : public argos::CCI_Controller {
 public:
  /**
   * @brief This structure holds data about food collecting by the robots
   */
  struct food_data {
    bool has_item;      // true when the robot is carrying a food item
    size_t curr_item_idx;    // the index of the current food item in the array of available food items
    size_t cum_items; // the total number of food items carried by this robot during the experiment

    void reset(void) {
      has_item = false;
      curr_item_idx = -1;
      cum_items = 0;
    }
  };

  social_foraging_controller(void);
  virtual ~social_foraging_controller() {}

  bool is_resting(void) { return m_fsm->is_resting(); }
  /*
   * @brief Initialize the controller.
   *
   * @param t_node Points to the <parameters> section in the XML file in the
   *               <controllers><social_foraging_controller_controller> section.
   */
  virtual void Init(argos::TConfigurationNode& t_node);

  /*
   * @brief Called once every time step; length set in the XML file.
   */
  virtual void ControlStep();

  /*
   * @brief Reset controller to its state right after the Init().
   */
  virtual void Reset(void);

  /*
   * @brief Cleanup whatever was done by Init().
   */
  virtual void Destroy() {}

  /*
   * Returns the food data
   */
  inline struct food_data& get_food_data() {
    return m_food_stats;
  }

 private:
  social_foraging_controller(const social_foraging_controller& fsm) = delete;
  social_foraging_controller& operator=(const social_foraging_controller& fsm) = delete;

  /* The random number generator */
  argos::CRandom::CRNG* m_rng;

  /* The controller state information */
  parameter_parser m_parser;
  std::shared_ptr<actuator_manager> m_actuators;
  std::shared_ptr<sensor_manager> m_sensors;
  std::unique_ptr<social_fsm> m_fsm;

  /* The food data */
  struct food_data m_food_stats;
};

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_SOCIAL_FORAGING_CONTROLLER_HPP_ */
