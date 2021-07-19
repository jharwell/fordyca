/**
 * \file lifecycle_metrics_data.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_CACHES_LIFECYCLE_METRICS_DATA_HPP_
#define INCLUDE_FORDYCA_METRICS_CACHES_LIFECYCLE_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/metrics/base_metrics_data.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, metrics, caches, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct lifecycle_metrics_data
 * \ingroup metrics caches detail
 *
 * \brief Container for holding collected statistics of \ref
 * lifecycle_metrics.
 */
struct lifecycle_metrics_data {
  size_t created{0};
  size_t depleted{0};
  size_t discarded{0};
  rtypes::timestep depletion_sum{0};
};

NS_END(detail);

struct lifecycle_metrics_data : public rmetrics::base_metrics_data {
  detail::lifecycle_metrics_data interval{};
  detail::lifecycle_metrics_data cum{};
};

NS_END(caches, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_CACHES_LIFECYCLE_METRICS_DATA_HPP_ */
