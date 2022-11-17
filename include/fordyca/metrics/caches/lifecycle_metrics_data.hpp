/**
 * \file lifecycle_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/metrics/base_data.hpp"
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

struct lifecycle_metrics_data : public rmetrics::base_data {
  detail::lifecycle_metrics_data interval{};
  detail::lifecycle_metrics_data cum{};
};

NS_END(caches, metrics, fordyca);

