/**
 * @file footbot_foraging.hpp
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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief  A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotForaging : public argos::CCI_Controller {

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

 public:
  CFootBotForaging(void);

  /* Class destructor. */
  virtual ~CFootBotForaging() {}

  /*
   * This function initializes the controller.
   * The 't_node' variable points to the <parameters> section in the XML
   * file in the <controllers><footbot_foraging_controller> section.
   */
  virtual void Init(argos::TConfigurationNode& t_node);

  /*
   * This function is called once every time step.
   * The length of the time step is set in the XML file.
   */
  virtual void ControlStep();

  /*
   * This function resets the controller to its state right after the
   * Init().
   * It is called when you press the reset button in the GUI.
   */
  virtual void Reset();

  /*
   * Called to cleanup what done by Init() when the experiment finishes.
   * In this example controller there is no need for clean anything up,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Destroy() {}

  /*
   * Returns true if the robot is currently exploring.
   */
  inline bool IsExploring() const {
    return m_sStateData.State == SStateData::STATE_EXPLORING;
  }

  /*
   * Returns true if the robot is currently resting.
   */
  inline bool IsResting() const {
    return m_sStateData.State == SStateData::STATE_RESTING;
  }

  /*
   * Returns true if the robot is currently returning to the nest.
   */
  inline bool IsReturningToNest() const {
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
  state_machine state_;
  parameter_parser params_;
  actuator_manager actuators_;
  sensor_manager sensors_;

  /* The food data */
  SFoodData m_sFoodData;

};

} /* namespace fordyca */

#endif /* INCLUDE_FORDYCA_FOOTBOT_FORAGING_HPP_ */
