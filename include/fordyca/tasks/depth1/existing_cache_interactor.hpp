/**
 * @file existing_cache_interactor.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_TASKS_DEPTH1_EXISTING_CACHE_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_TASKS_DEPTH1_EXISTING_CACHE_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace events {
class cached_block_pickup;
class cache_block_drop;
class cache_vanished;
} // namespace events

namespace visitor = rcppsw::patterns::visitor;

NS_START(tasks, depth1);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class existing_cache_interactor
 * @ingroup tasks depth1
 *
 * @brief Interactor specifying the event visit set for all foraging tasks that
 * interact with existing caches in FORDYCA.
 */
class existing_cache_interactor
    : public visitor::polymorphic_accept_set<events::cache_block_drop,
                                             events::cached_block_pickup,
                                             events::cache_vanished> {};

NS_END(depth1, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_DEPTH1_EXISTING_CACHE_INTERACTOR_HPP_ */
