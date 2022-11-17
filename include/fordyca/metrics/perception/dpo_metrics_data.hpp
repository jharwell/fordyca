/**
 * \file dpo_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_data.hpp"
#include "rcppsw/al/multithread.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, metrics, perception, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct dpo_metrics_data
 * \ingroup metrics perception detail
 *
 * \brief Container for holding collected statistics of \ref
 * dpo_metrics. Must be atomic so counts are valid in parallel metric
 * collection contexts.
 */
struct dpo_metrics_data {
  ral::mt_size_t  robot_count{0};
  ral::mt_size_t  known_blocks{0};
  ral::mt_size_t  known_caches{0};
  ral::mt_double_t block_density_sum{0.0};
  ral::mt_double_t cache_density_sum{0.0};
};

NS_END(detail);

struct dpo_metrics_data : public rmetrics::base_data {
  detail::dpo_metrics_data interval{};
  detail::dpo_metrics_data cum{};
};

NS_END(perception, metrics, fordyca);
