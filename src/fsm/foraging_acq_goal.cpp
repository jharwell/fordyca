/**
 * \file foraging_acq_goal.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/foraging_acq_goal.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool operator==(const csmetrics::goal_acq_metrics::goal_type& goal1,
                const foraging_acq_goal& goal2) {
  return goal1.v() == rcppsw::as_underlying(goal2);
}

bool operator==(const foraging_acq_goal& goal1,
                const csmetrics::goal_acq_metrics::goal_type& goal2) {
  return goal2.v() == rcppsw::as_underlying(goal1);
}

bool operator!=(const csmetrics::goal_acq_metrics::goal_type& goal1,
                const foraging_acq_goal& goal2) {
  return goal1.v() != rcppsw::as_underlying(goal2);
}

bool operator!=(const foraging_acq_goal& goal1,
                const csmetrics::goal_acq_metrics::goal_type& goal2) {
  return goal2.v() != rcppsw::as_underlying(goal1);
}

csmetrics::goal_acq_metrics::goal_type
to_goal_type(const foraging_acq_goal& goal) {
  return csmetrics::goal_acq_metrics::goal_type(rcppsw::as_underlying(goal));
}

NS_END(fsm, fordyca);
