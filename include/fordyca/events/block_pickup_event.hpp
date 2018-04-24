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

namespace fsm { namespace depth1 { class cached_block_to_nest_fsm; } }
namespace visitor = rcppsw::patterns::visitor;
namespace controller { namespace depth1 {
class foraging_controller;
}} // namespace controller::depth1

namespace representation {
class arena_map;
class perceived_arena_map;
class block;
} // namespace representation

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_pickup_event
 * @ingroup events
 *
 * @brief Interface specifying the minimum set of classes that any event
 * involving picking up a block will need to visit.
 */
class block_pickup_event
    : public visitor::visit_set<fsm::depth1::cached_block_to_nest_fsm,
                                representation::arena_map,
                                representation::perceived_arena_map,
                                representation::block,
                                controller::depth1::foraging_controller> {};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_PICKUP_EVENT_HPP_ */
