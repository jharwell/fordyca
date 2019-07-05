/**
 * @file swarm_iterator.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_SWARM_ITERATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_SWARM_ITERATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/base_controller.hpp"
#include "fordyca/support/base_loop_functions.hpp"
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
struct swarm_iterator {
  template <typename TFunction>
  static void controllers(const base_loop_functions* const lf,
                          const TFunction& cb) {
    for (auto& [name, robotp] : lf->GetSpace().GetEntitiesByType("foot-bot")) {
      auto* robot = argos::any_cast<argos::CFootBotEntity*>(robotp);
      auto* controller = dynamic_cast<controller::base_controller*>(
          &robot->GetControllableEntity().GetController());
      cb(controller);
    } /* for(...) */
  }
  template <typename TFunction>
  static void robots(const base_loop_functions* const lf, const TFunction& cb) {
    for (auto& [name, robotp] : lf->GetSpace().GetEntitiesByType("foot-bot")) {
      auto* robot = argos::any_cast<argos::CFootBotEntity*>(robotp);
      cb(robot);
    } /* for(...) */
  }
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_SWARM_ITERATOR_HPP_ */
