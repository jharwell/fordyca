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
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include <boost/shared_ptr.hpp>
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/controller/random_foraging_fsm.hpp"
#include "fordyca/controller/sensor_manager.hpp"
#include "fordyca/controller/actuator_manager.hpp"

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
class random_foraging_controller : public argos::CCI_Controller,
                                   public rcppsw::common::er_client,
                                   public visitor::visitable<random_foraging_controller> {
 public:
  random_foraging_controller(void);

  bool is_searching_for_block(void) const { return m_fsm->is_searching_for_block(); }
  bool is_returning(void) const { return m_fsm->is_returning(); }
  bool is_avoiding_collision(void) const { return m_fsm->is_avoiding_collision(); }
  bool in_nest(void) const { return m_sensors->in_nest(); }
  bool block_detected(void) const { return m_sensors->block_detected(); }
  void display_id(bool display_id) { m_display_id = display_id; }
  bool display_id(void) const { return m_display_id; }

  /*
   * @brief Initialize the controller.
   *
   * @param t_node Points to the <parameters> section in the XML file in the
   *               <controllers><random_foraging_controller_controller> section.
   */
  virtual void Init(argos::TConfigurationNode& t_node);

  /*
   * @brief Called once every time step; length set in the XML file.
   *
   * Since the FSM does most of the work, this function just tells it run.
   */
  virtual void ControlStep(void) { m_fsm->run(); }

  /*
   * @brief Reset controller to its state right after the Init().
   */
  virtual void Reset(void);

  /*
   * @brief Cleanup whatever was done by Init().
   */
  virtual void Destroy(void) {}

  /**
   * @brief Return if the robot is currently carrying a block.
   */
  bool is_carrying_block(void) const { return nullptr != m_block; }
  void publish_fsm_event(foraging_signal::type signal);

  /**
   * @brief Return the block robot is carrying, or NULL if the robot is not
   * currently carrying a block.
   */
  representation::block* block(void) const { return m_block; }
  void block(representation::block* block) { m_block = block; }

 protected:
  const std::shared_ptr<sensor_manager>& sensors(void) const { return m_sensors; }
  const std::shared_ptr<actuator_manager>& actuators(void) const { return m_actuators; }
  const std::shared_ptr<rcppsw::common::er_server>& server(void) const { return m_server; }

 private:
  random_foraging_controller(const random_foraging_controller& other) = delete;
  random_foraging_controller& operator=(const random_foraging_controller& other) = delete;

  /** Should the ID of the robot be displayed during visualization?  */
  bool                                       m_display_id;

  /**
   * The current block that the robot is carrying, or NULL if the robot is not
   * currently carrying a block.
   */
  representation::block*                     m_block;

  /** The er_server that this and all derived classes will use */
  std::shared_ptr<rcppsw::common::er_server> m_server;
  std::shared_ptr<actuator_manager>          m_actuators;
  std::shared_ptr<sensor_manager>            m_sensors;
  std::unique_ptr<random_foraging_fsm>       m_fsm;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_RANDOM_FORAGING_CONTROLLER_CONTROLLER_HPP_ */
