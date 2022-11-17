/**
 * \file utils.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

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

