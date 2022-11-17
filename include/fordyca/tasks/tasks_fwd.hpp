/**
 * \file tasks_fwd.hpp
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
NS_START(fordyca, tasks);
class base_foraging_task;
namespace d0 {
class generalist;
}
namespace d1 {
class collector;
class harvester;
} // namespace d1
namespace d2 {
class cache_starter;
class cache_finisher;
class cache_transferer;
class cache_collector;
} // namespace d2

NS_END(fordyca, tasks);
