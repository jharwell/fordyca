n/**
 * @file social_foraging.hpp
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

#ifndef INCLUDE_FORDYCA_FOOTBOT_FORAGING_HPP_
#define INCLUDE_FORDYCA_FOOTBOT_FORAGING_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include "fordyca/fordyca_params.hpp"
#include "fordyca/state_machine.hpp"
#include "fordyca/parameter_parser.hpp"
#include "fordyca/sensor_manager.hpp"
#include "fordyca/actuator_manager.hpp"
#include "fordyca/social_foraging_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controllers);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief  A controller is simply an implementation of the CCI_Controller class.
 */
class foraging_base : public argos::CCI_Controller {

 public:

  /**
   * @brief This structure holds data about food collecting by the robots
   */
  struct SFoodData {
    bool has_item_;      // true when the robot is carrying a food item
    size_t curr_item_idx_;    // the index of the current food item in the array of available food items
    size_t cum_items_; // the total number of food items carried by this robot during the experiment

    SFoodData();
    void Reset();
  };

  foraging_base(void);
  virtual ~foraging_base() {}

  /*
   * @brief Initialize the controller.
   *
   * @param t_node Points to the <parameters> section in the XML file in the
   *               <controllers><footbot_foraging_controller> section.
   */
  virtual void Init(argos::TConfigurationNode& t_node);

  /*
   * @brief Called once every time step; length set in the XML file.
   */
  virtual void ControlStep();

  /*
   * @brief Reset controller to its state right after the Init().
   */
  virtual void Reset();

  /*
   * @brief Cleanup whatever was done by Init().
   */
  virtual void Destroy() {}

  /*
   * Returns true if the robot is currently exploring.
   */
  inline bool is_exploring() const {
    return m_sStateData.State == SStateData::STATE_EXPLORING;
  }

  /*
   * Returns true if the robot is currently resting.
   */
  inline bool is_resting() const {
    return m_sStateData.State == SStateData::STATE_RESTING;
  }

  /*
   * Returns true if the robot is currently returning to the nest.
   */
  inline bool is_returning_to_rest() const {
    return m_sStateData.State == SStateData::STATE_RETURN_TO_NEST;
  }

  /*
   * Returns the food data
   */
  inline SFoodData& GetFoodData() {
    return m_sFoodData;
  }

 private:

  /*
   * Executes the resting state.
   */
  void Rest();

  /*
   * Executes the exploring state.
   */
  void Explore();

  /*
   * Executes the return to nest state.
   */
  void ReturnToNest();

 private:

  /* The random number generator */
  argos::CRandom::CRNG* m_pcRNG;

  /* Used in the social rule to communicate the result of the last
   * exploration attempt */
  enum ELastExplorationResult {
    LAST_EXPLORATION_NONE = 0,    // nothing to report
    LAST_EXPLORATION_SUCCESSFUL,  // the last exploration resulted in a food item found
    LAST_EXPLORATION_UNSUCCESSFUL // no food found in the last exploration
  } m_eLastExplorationResult;

  /* The controller state information */
  social_foraging_fsm fsm_;
  parameter_parser params_;
  actuator_manager actuators_;
  sensor_manager sensors_;

  /* The food data */
  SFoodData m_sFoodData;

};

NS_END(controllers, fordyca);

#endif /* INCLUDE_FORDYCA_FOOTBOT_FORAGING_HPP_ */
