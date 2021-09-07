/**
 * \file foraging_acq_goal.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
