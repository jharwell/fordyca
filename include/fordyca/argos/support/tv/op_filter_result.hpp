/**
 * \file op_filter_result.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/type_uuid.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/argos/support/tv/op_filter_status.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
struct op_filter_result {
  op_filter_status status{op_filter_status::ekROBOT_INTERNAL_UNREADY};
  rtypes::type_uuid     id{rtypes::constants::kNoUUID};
};


NS_END(tv, argos support, argos, fordyca);

