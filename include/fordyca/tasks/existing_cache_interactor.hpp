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

#ifndef INCLUDE_FORDYCA_TASKS_EXISTING_CACHE_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_TASKS_EXISTING_CACHE_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/metrics/fsm/block_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/block_transport_metrics.hpp"
#include "fordyca/metrics/fsm/cache_acquisition_metrics.hpp"
#include "fordyca/tasks/argument.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"
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

NS_START(tasks);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class existing_cache_interactor
 * @ingroup tasks
 *
 * @brief Interactor specifying the event visit set for all foraging tasks that
 * interact with existing caches in FORDYCA.
 */
class existing_cache_interactor
    : public metrics::fsm::cache_acquisition_metrics,
      public visitor::polymorphic_visitable<existing_cache_interactor,
                                            events::cached_block_pickup,
                                            events::cache_block_drop,
                                            events::cache_vanished> {};

NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_EXISTING_CACHE_INTERACTOR_HPP_ */
