/**
 * @file motion_throttling_handler.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_MOTION_THROTTLING_HANDLER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_MOTION_THROTTLING_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/nsalias.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace control {
class waveform;
namespace config {
struct waveform_config;
} /* namespace config */
}} // namespace rcppsw::control

NS_START(fordyca, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class motion_throttling_handler
 * @ingroup fordyca support tv
 *
 * @brief Handler for a type of motion_throttling for the robot actuators
 */
class motion_throttling_handler {
 public:
  explicit motion_throttling_handler(const rct::config::waveform_config* config);
  ~motion_throttling_handler(void);

  /**
   * @brief Get the applied amount of motion_throttling (a percentage between 0
   * and 1) that should be applied to the robot. Only > 0 if the robot is
   * actually carrying a block.
   */
  double active_throttle(void) const { return m_active; }

  /**
   * @brief Get the current amount of motion_throttling (a percentage between 0
   * and 1) that is configured for block carry and being applied to the robot
   * (regardless if it is active or not).
   */
  double applied_throttle(void) const { return m_applied; }

  /**
   * @brief Set the current block carry state.
   */
  void toggle(bool en) { m_en = en; }

  /**
   * @brief Update the actuators in accordance with the current motion_throttling
   * configuration and timestep.
   */
  void update(rtypes::timestep t);

 private:
  /* clang-format off */
  bool                           m_en{false};
  double                         m_active{0.0};
  double                         m_applied{0.0};
  std::unique_ptr<rct::waveform> m_waveform;
  /* clang-format off */
};

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_MOTION_THROTTLING_HANDLER_HPP_ */
