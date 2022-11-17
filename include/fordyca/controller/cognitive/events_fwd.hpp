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
NS_START(fordyca, controller, cognitive);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
namespace d0::events {
class block_vanished;
class free_block_pickup;
class free_block_drop;

class nest_block_drop;
}
namespace d1::events {
class block_vanished;
class free_block_pickup;
class free_block_drop;

class cache_block_drop;
class cached_block_pickup;
class cache_vanished;

class nest_block_drop;
}
namespace d2::events {
class block_vanished;
class free_block_pickup;
class free_block_drop;

class cache_block_drop;
class cached_block_pickup;
class cache_vanished;

class nest_block_drop;

class block_proximity;
class cache_proximity;
}

NS_END(events, controller, cognitive);

