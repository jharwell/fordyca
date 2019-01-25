/**
 * @file block_pickup_event.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_EVENTS_BLOCK_PICKUP_EVENT_HPP_
#define INCLUDE_FORDYCA_EVENTS_BLOCK_PICKUP_EVENT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace representation {
class base_block;
} // namespace representation

namespace ds {
class arena_map;
class dpo_semantic_map;
class dpo_store;
} // namespace ds

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_pickup_event
 * @ingroup events
 *
 * @brief Interface specifying the core class of classes any action involving
 * dropping a block will need to visit (think data structures).
 */
class block_pickup_event
    : public visitor::visit_set<ds::arena_map,
                                ds::dpo_semantic_map,
                                ds::dpo_store,
                                representation::base_block> {};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_PICKUP_EVENT_HPP_ */
