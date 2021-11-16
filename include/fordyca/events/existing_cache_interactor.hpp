/**
 * \file existing_cache_interactor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_EVENTS_EXISTING_CACHE_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_EVENTS_EXISTING_CACHE_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/controller/cognitive/events_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \class existing_cache_interactor
 * \ingroup tasks d1
 *
 * \brief Interactor specifying the event visit set for all foraging tasks that
 * interact with existing caches in FORDYCA.
 */
class existing_cache_interactor : public rpvisitor::polymorphic_accept_set<
  fccd1::events::cache_block_drop,
  fccd1::events::cached_block_pickup,
  fccd1::events::cache_vanished,

  fccd2::events::cache_block_drop,
  fccd2::events::cached_block_pickup,
  fccd2::events::cache_vanished
  > {};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_EXISTING_CACHE_INTERACTOR_HPP_ */
