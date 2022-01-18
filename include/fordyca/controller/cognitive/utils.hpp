/**
 * \file utils.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_UTILS_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_UTILS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"
#include "fordyca/controller/cognitive/d0/events/free_block_pickup.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, controller, cognitive);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
NS_START(free_block_pickup);

template<typename TController, typename TPerceptionModel>
void controller_visit(TController& controller) {
  controller.ndc_uuid_push();

  base_pickup::visit(*controller.perception()->template model<TPerceptionModel>());
  visit(static_cast<ccontroller::block_carrying_controller&>(controller));
  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_uuid_pop();
}

NS_END(cognitive, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_UTILS_HPP_ */
