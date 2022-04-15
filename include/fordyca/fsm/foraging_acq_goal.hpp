/**
 * \file foraging_acq_goal.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief Representats the different types of objects robots can acquire as part
 * of the foraging process.
 */
enum class foraging_acq_goal {
  ekNONE = -1,
  ekBLOCK,
  ekNEST,
  ekCACHE_SITE,
  ekNEW_CACHE,
  ekEXISTING_CACHE
};

/*******************************************************************************
 * Operators
 ******************************************************************************/
bool operator==(const csmetrics::goal_acq_metrics::goal_type& goal1,
                const foraging_acq_goal& goal2) RCPPSW_PURE;

bool operator==(const foraging_acq_goal& goal1,
                const csmetrics::goal_acq_metrics::goal_type& goal2) RCPPSW_PURE;

bool operator!=(const csmetrics::goal_acq_metrics::goal_type& goal1,
                const foraging_acq_goal& goal2) RCPPSW_PURE;

bool operator!=(const foraging_acq_goal& goal1,
                const csmetrics::goal_acq_metrics::goal_type& goal2) RCPPSW_PURE;

csmetrics::goal_acq_metrics::goal_type
to_goal_type(const foraging_acq_goal& goal);

NS_END(fsm, fordyca);
