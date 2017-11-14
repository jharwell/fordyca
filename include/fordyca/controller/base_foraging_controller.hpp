/**
 * @file base_foraging_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_BASE_FORAGING_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_BASE_FORAGING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/control_interface/ci_controller.h>
#include "rcppsw/common/er_client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation { class block; class line_of_sight; }

NS_START(controller);

class actuator_manager;
class base_foraging_sensors;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class base_foraging_controller : public argos::CCI_Controller,
                                 public rcppsw::common::er_client {
 public:
  base_foraging_controller(void);
  virtual ~base_foraging_controller(void) {}

  void display_los(bool display_los) { m_display_los = display_los; }

  /**
   * @brief If TRUE, then the robot should display its approxibate LOS as a
   * circle on the ground during simulation.
   */
  bool display_los(void) const { return m_display_los; }

  /**
   * @brief Set whether or not a robot is supposed to display it's ID above its
   * head during simulation.
   */
  void display_id(bool display_id) { m_display_id = display_id; }

  /**
   * @brief Return whether or not a robot is supposed to display it's ID above
   * its head during simulation.
   */
  bool display_id(void) const { return m_display_id; }

  /**
   * @brief If TRUE, the robot is currently at least most of the way in the nest.
   */
  bool in_nest(void) const;

  virtual const representation::line_of_sight* los(void) const { return nullptr; }

  /**
   * @brief Return if the robot is currently carrying a block.
   */
  bool is_carrying_block(void) const { return nullptr != m_block; }

  /**
   * @brief Return the block robot is carrying, or NULL if the robot is not
   * currently carrying a block.
   */
  representation::block* block(void) const { return m_block; }
  void block(representation::block* block) { m_block = block; }

  /**
   * @brief If TRUE, then the robot thinks that it is on top of a block. Note
   * that this may be a false positive...
   */
  bool block_detected(void) const;

  void Init(argos::TConfigurationNode& node) override;

  /*
   * @brief Reset controller to its state right after the Init().
   */
  void Reset(void) override;

  /*
   * @brief Cleanup whatever was done by Init().
   */
  void Destroy(void) override {}

 protected:
  const std::shared_ptr<actuator_manager>& actuators(void) const { return m_actuators; }
  const std::shared_ptr<rcppsw::common::er_server>& server(void) const { return m_server; }
  const std::shared_ptr<base_foraging_sensors>& sensors(void) const { return m_sensors; }

 private:
  base_foraging_controller(const base_foraging_controller& other) = delete;
  base_foraging_controller& operator=(const base_foraging_controller& other) = delete;

  bool m_display_los;
  bool m_display_id;

  /**
   * The current block that the robot is carrying, or NULL if the robot is not
   * currently carrying a block.
   */
  representation::block*                        m_block;
  std::shared_ptr<actuator_manager>             m_actuators;
  std::shared_ptr<base_foraging_sensors> m_sensors;
  std::shared_ptr<rcppsw::common::er_server>    m_server;
};

NS_END(fordyca, controller);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BASE_FORAGING_CONTROLLER_HPP_ */
