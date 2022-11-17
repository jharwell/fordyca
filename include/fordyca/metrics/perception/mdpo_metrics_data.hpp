/**
 * \file mdpo_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <array>

#include "rcppsw/metrics/base_data.hpp"
#include "rcppsw/al/multithread.hpp"

#include "cosm/fsm/cell2D_state.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, metrics, perception, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct mdpo_metrics_data
 * \ingroup metrics perception detail
 *
 * \brief Container for holding collected statistics of \ref
 * mdpo_metrics. Must be atomic so counts are valid in parallel metric
 * collection contexts.
 */
struct mdpo_metrics_data {
  std::array<ral::mt_size_t, cfsm::cell2D_state::ekST_MAX_STATES> states;
  ral::mt_double_t known_percent{0.0};
  ral::mt_double_t unknown_percent{0.0};
  ral::mt_size_t   robots{0};
};

NS_END(detail);

struct mdpo_metrics_data : public rmetrics::base_data {
  detail::mdpo_metrics_data interval{};
  detail::mdpo_metrics_data cum{};
};

NS_END(perception, metrics, fordyca);

