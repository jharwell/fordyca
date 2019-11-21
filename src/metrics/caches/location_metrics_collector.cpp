/**
 * \file location_metrics_collector.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/caches/location_metrics_collector.hpp"

#include "fordyca/metrics/caches/location_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void location_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const location_metrics&>(metrics);
  inc_total_count();

  for (size_t i = 0; i < xsize(); ++i) {
    for (size_t j = 0; j < ysize(); ++j) {
      inc_cell_count(i, j, m.location() == rmath::vector2u(i, j));
    } /* for(j..) */
  }   /* for(i..) */
} /* collect() */

NS_END(caches, metrics, fordyca);
