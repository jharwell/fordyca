/**
 * \file events_fwd.hpp
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

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, controller, reactive);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
namespace d0::events {
class block_vanished;
class free_block_pickup;
class nest_block_drop;
}

NS_END(events, controller, reactive);

