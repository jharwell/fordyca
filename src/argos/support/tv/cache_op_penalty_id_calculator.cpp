/**
 * \file cache_op_penalty_id_calculator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/tv/cache_op_penalty_id_calculator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cache_op_penalty_id_calculator::cache_op_penalty_id_calculator(void)
    : ER_CLIENT_INIT("fordyca.argos.support.tv.cache_op_penalty_id_calculator") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rtypes::type_uuid
cache_op_penalty_id_calculator::operator()(cache_op_src src,
                                           op_filter_result filter) const {
  rtypes::type_uuid id = rtypes::constants::kNoUUID;
  switch (src) {
    case cache_op_src::ekEXISTING_CACHE_DROP:
    case cache_op_src::ekEXISTING_CACHE_PICKUP:
      ER_ASSERT(rtypes::constants::kNoUUID != filter.id, "Robot not in cache?");
      id = filter.id;
    default:
      break;
  }
  return id;
} /* penalty_id_calc() */

NS_END(tv, support, argos, fordyca);
