/**
 * \file block_op_penalty_id_calculator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/tv/block_op_penalty_id_calculator.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_op_penalty_id_calculator::block_op_penalty_id_calculator(
    const carena::caching_arena_map* map)
    : ER_CLIENT_INIT("fordyca.argos.support.tv.block_op_penalty_id_calculator"),
      mc_map(map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rtypes::type_uuid block_op_penalty_id_calculator::operator()(
    const controller::foraging_controller& controller,
    const block_op_src& src,
    const op_filter_result& filter) const {
  rtypes::type_uuid id = rtypes::constants::kNoUUID;
  switch (src) {
    case block_op_src::ekFREE_PICKUP:
      /*
       * We don't call the base class function for this because of the lack of
       * locking around the BOTH block op filtering and penalty ID calculation
       * to make them atomic, and the conditions for free pickup might not be
       * met anymore. If they are not, it will be caught after the pickup
       * penalty has been served.
       */
      id = filter.id;
      break;
    case block_op_src::ekNEST_DROP:
      id = decoratee().from_nest_drop(controller.block());
      break;
    case block_op_src::ekCACHE_SITE_DROP:
      ER_ASSERT(nullptr != controller.block() &&
                    rtypes::constants::kNoUUID != controller.block()->id(),
                "Robot not carrying block?");
      id = controller.block()->id();
      break;
    case block_op_src::ekNEW_CACHE_DROP:
      ER_ASSERT(nullptr != controller.block() &&
                    rtypes::constants::kNoUUID != controller.block()->id(),
                "Robot not carrying block?");
      id = controller.block()->id();
      break;
    default:
      break;
  }
  return id;
} /* penalty_id_calc() */

NS_END(tv, support, argos, fordyca);
