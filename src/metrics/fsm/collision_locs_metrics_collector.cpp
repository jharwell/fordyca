/**
 * @file collision_locs_metrics_collector.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/metrics/fsm/collision_locs_metrics_collector.hpp"
#include "fordyca/metrics/fsm/collision_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
uint collision_locs_metrics_collector::collect_cell(
    const rmetrics::base_metrics& metrics,
    const rmath::vector2u& coord) const {
  auto& m = dynamic_cast<const collision_metrics&>(metrics);

  if (m.in_collision_avoidance()) {
    return static_cast<uint>(m.avoidance_loc() == coord);
  } else {
    return 0;
  }
} /* collect_cell() */

NS_END(fsm, metrics, fordyca);
