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
#include "fordyca/controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rtypes::type_uuid block_op_penalty_id_calculator::operator()(
    const controller::foraging_controller& controller,
    block_op_src src) const {
  rtypes::type_uuid id = rtypes::constants::kNoUUID;
  switch (src) {
    case block_op_src::ekFREE_PICKUP:
      id = decoratee().from_free_pickup(controller.pos2D(),
                                        controller.entity_acquired_id(),
                                        mc_map);
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
