/**
 * \file block_pickup_base_visit_set.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_EVENTS_BLOCK_PICKUP_BASE_VISIT_SET_HPP_
#define INCLUDE_FORDYCA_EVENTS_BLOCK_PICKUP_BASE_VISIT_SET_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/mpl/typelist.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block2D;
} // namespace cosm::repr

namespace cosm::foraging::ds {
class arena_map;
} /* namespace cosm::foraging::ds */

NS_START(fordyca);

namespace ds {
class dpo_semantic_map;
class dpo_store;
} // namespace ds

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \ingroup events detail
 *
 * \brief Interface specifying the core class of classes any action involving
 * dropping a block will need to visit (think data structures).
 */
using block_pickup_base_visit_typelist = rmpl::typelist<cfds::arena_map,
                                                        ds::dpo_semantic_map,
                                                        ds::dpo_store,
                                                        crepr::base_block2D>;
NS_END(detail, events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_PICKUP_BASE_VISIT_SET_HPP_ */
