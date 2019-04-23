/**
 * @file dpo_semantic_map.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/params/perception/perception_params.hpp"
#include "fordyca/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_semantic_map::dpo_semantic_map(
    const params::perception::perception_params* c_params,
    const std::string& robot_id)
    : ER_CLIENT_INIT("fordyca.ds.dpo_semantic_map"),
      decorator(c_params, robot_id),
      m_store(&c_params->pheromone) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool dpo_semantic_map::cache_remove(
    const std::shared_ptr<repr::base_cache>& victim) {
  if (m_store.cache_remove(victim)) {
    ER_DEBUG("Updating cell@%s for removed cache",
             victim->discrete_loc().to_str().c_str());
    events::cell_empty_visitor op(victim->discrete_loc());
    op.visit(decoratee().access<occupancy_grid::kCell>(victim->discrete_loc()));
    return true;
  }
  return false;
} /* cache_remove() */

bool dpo_semantic_map::block_remove(
    const std::shared_ptr<repr::base_block>& victim) {
  if (m_store.block_remove(victim)) {
    ER_DEBUG("Updating cell@%s for removed block",
             victim->discrete_loc().to_str().c_str());
    events::cell_empty_visitor op(victim->discrete_loc());
    op.visit(access<occupancy_grid::kCell>(victim->discrete_loc()));
    return true;
  }
  return false;
} /* block_remove() */

void dpo_semantic_map::decay_all(void) {
  decoratee().update();
  m_store.decay_all();

  for (auto& b : m_store.blocks().const_values_range()) {
    const rmath::vector2u& loc = b.ent()->discrete_loc();
    rswarm::pheromone_density& map_density =
        decoratee().access<occupancy_grid::kPheromone>(loc);

    ER_ASSERT(std::fabs((map_density - b.density()).last_result()) <=
                  std::numeric_limits<double>::epsilon(),
              "FATAL: Map density@%s and DP block%d density disagree: %f vs %f",
              loc.to_str().c_str(),
              b.ent()->id(),
              map_density.last_result(),
              b.density().last_result());
  } /* for(&b..) */

  for (auto&& c : m_store.caches().const_values_range()) {
    const rmath::vector2u& loc = c.ent()->discrete_loc();
    rswarm::pheromone_density& map_density =
        decoratee().access<occupancy_grid::kPheromone>(loc);

    ER_ASSERT(std::fabs((map_density - c.density()).last_result()) <=
                  std::numeric_limits<double>::epsilon(),
              "FATAL: Map density@%s and DP cache%d density disagree: %f vs %f",
              loc.to_str().c_str(),
              c.ent()->id(),
              map_density.last_result(),
              c.density().last_result());
  } /* for(&c..) */
} /* decay_all() */

NS_END(ds, fordyca);
