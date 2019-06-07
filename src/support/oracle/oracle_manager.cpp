/**
 * @file oracle_manager.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/support/oracle/oracle_manager.hpp"
#include "fordyca/config/oracle/oracle_manager_config.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/support/oracle/entities_oracle.hpp"
#include "fordyca/support/oracle/tasking_oracle.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, oracle);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
oracle_manager::oracle_manager(
    const config::oracle::oracle_manager_config* const config)
    : m_entities(rcppsw::make_unique<class entities_oracle>(&config->entities)),
      m_tasking(nullptr) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void oracle_manager::tasking_oracle(std::unique_ptr<class tasking_oracle> o) {
  m_tasking = std::move(o);
} /* tasking_oracle */

void oracle_manager::update(ds::arena_map* const map) {
  if (m_entities->blocks_enabled()) {
    entities_oracle::variant_vector_type v;
    for (auto& b : map->blocks()) {
      if (-1 == b->robot_id()) { /* don't include blocks robot's are carrying */
        v.push_back(b);
      }
    } /* for(&b..) */
    m_entities->set_blocks(v);
  }
  if (m_entities->caches_enabled()) {
    entities_oracle::variant_vector_type v;
    for (auto& b : map->caches()) {
      v.push_back(b);
    } /* for(&b..) */
    m_entities->set_caches(v);
  }
} /* update() */

NS_END(oracle, support, fordyca);
