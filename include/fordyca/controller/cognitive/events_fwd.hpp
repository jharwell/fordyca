/**
 * \file events_fwd.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_EVENTS_FWD_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_EVENTS_FWD_HPP_

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

#endif /* INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_EVENTS_FWD_HPP_ */
