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
#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "fordyca/controller/saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class explore_behavior
 * @ingroup fsm
 *
 * @brief Base class for different exploration behaviors that robots can exhibit
 * when looking for stuff.
 */
class explore_behavior : public rcppsw::er::client {
 public:
  explore_behavior(const std::shared_ptr<rcppsw::er::server>& server,
                   const std::shared_ptr<controller::saa_subsystem>& saa)
      : client(server), m_saa(saa) {}

  virtual ~explore_behavior(void) = default;

  explore_behavior(const explore_behavior& fsm) = delete;
  explore_behavior& operator=(const explore_behavior& fsm) = delete;

  /**
   * @brief Defines the actual robot behavior during exploration.
   */
  virtual void execute(void) = 0;

 protected:
  std::shared_ptr<controller::saa_subsystem> saa_subsystem(void) { return m_saa; }
  std::shared_ptr<const controller::saa_subsystem> saa_subsystem(void) const { return m_saa; }

 private:
  std::shared_ptr<controller::saa_subsystem> m_saa;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_EXPLORE_BEHAVIOR_HPP_ */
