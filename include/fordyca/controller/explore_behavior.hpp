/**
 * @file explore_behavior.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_EXPLORE_BEHAVIOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_EXPLORE_BEHAVIOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/metrics/fsm/collision_metrics.hpp"
#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class explore_behavior
 * @ingroup fordyca fsm
 *
 * @brief Base class for different exploration behaviors that robots can exhibit
 * when looking for stuff.
 */
class explore_behavior : public rcppsw::er::client<explore_behavior>,
                         public metrics::fsm::collision_metrics {
 public:
  explicit explore_behavior(controller::saa_subsystem* const saa)
      : ER_CLIENT_INIT("fordyca.controller.explore_behavior"), m_saa(saa) {}

  ~explore_behavior(void) override = default;

  explore_behavior(const explore_behavior& fsm) = delete;
  explore_behavior& operator=(const explore_behavior& fsm) = delete;

  /**
   * @brief Defines the actual robot behavior during exploration.
   */
  virtual void execute(void) = 0;

 protected:
  const controller::saa_subsystem* saa_subsystem(void) const { return m_saa; }
  controller::saa_subsystem* saa_subsystem(void) { return m_saa; }

 private:
  controller::saa_subsystem* const m_saa;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_EXPLORE_BEHAVIOR_HPP_ */
