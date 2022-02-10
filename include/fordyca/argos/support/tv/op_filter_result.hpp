/**
 * \file op_filter_result.hpp
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

