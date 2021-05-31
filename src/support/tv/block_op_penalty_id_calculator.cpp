/**
 * \file block_op_penalty_id_calculator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/tv/block_op_penalty_id_calculator.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "fordyca//controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_op_penalty_id_calculator::block_op_penalty_id_calculator(
    const carena::caching_arena_map* map)
    : ER_CLIENT_INIT("fordyca.support.tv.block_op_penalty_id_calculator"),
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

NS_END(tv, support, fordyca);
