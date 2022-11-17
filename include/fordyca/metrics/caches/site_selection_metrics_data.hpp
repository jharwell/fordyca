/**
 * \file site_selection_metrics_data.hpp
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

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, metrics, caches, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct site_selection_metrics_data
 * \ingroup metrics caches detail
 *
 * \brief Container for holding collected statistics of \ref
 * site_selection_metrics.
 */
struct site_selection_metrics_data {
  size_t n_successes{0};
  size_t n_fails{0};
  size_t nlopt_stopval{0};
  size_t nlopt_ftol{0};
  size_t nlopt_xtol{0};
  size_t nlopt_maxeval{0};
};

NS_END(detail);

struct site_selection_metrics_data : public rmetrics::base_data {
  detail::site_selection_metrics_data interval{};
  detail::site_selection_metrics_data cum{};
};

NS_END(caches, metrics, fordyca);

