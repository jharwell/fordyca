/**
 * @file forager.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/tasks/forager.hpp"
#include "fordyca/fsm/block_to_cache_fsm.hpp"
#include "fordyca/controller/sensor_manager.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/block_found.hpp"


/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double forager::calc_elapsed_time(double exec_time) const {
  return dynamic_cast<fsm::block_to_cache_fsm*>(polled_task::mechanism())->sensors()->tick() - exec_time;
} /* elapsed_time() */

void forager::accept(events::cache_block_drop &visitor) { visitor.visit(*this); }
void forager::accept(events::cache_found &visitor) { visitor.visit(*this); }
void forager::accept(events::free_block_pickup &visitor) { visitor.visit(*this); }
void forager::accept(events::block_found &visitor) { visitor.visit(*this); }

NS_END(tasks, fordyca);
