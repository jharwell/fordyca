/**
 * \file fsm_fwd.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);
class block_to_goal_fsm;
namespace d0 {
class crw_fsm;
class dpo_fsm;
class free_block_to_nest_fsm;
} // namespace d0

namespace d1 {
class cached_block_to_nest_fsm;
} // namespace d1

NS_END(fsm, fordyca);
