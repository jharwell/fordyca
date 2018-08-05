/**
 * @file cell2D.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell2D::cell2D(const std::shared_ptr<rcppsw::er::server>& server)
    : m_loc(), m_fsm(server) {
  m_fsm.init();
}

cell2D::cell2D(void) : m_loc(), m_fsm() { m_fsm.init(); }

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_pure std::shared_ptr<const representation::base_block> cell2D::block(
    void) const {
  return std::dynamic_pointer_cast<representation::base_block>(m_entity);
} /* block() */

__rcsw_pure std::shared_ptr<representation::base_block> cell2D::block(void) {
  return std::dynamic_pointer_cast<representation::base_block>(m_entity);
} /* block() */

__rcsw_pure std::shared_ptr<representation::base_cache> cell2D::cache(void) {
  return std::dynamic_pointer_cast<representation::base_cache>(m_entity);
} /* cache() */

__rcsw_pure const std::shared_ptr<representation::base_cache> cell2D::cache(
    void) const {
  return std::dynamic_pointer_cast<representation::base_cache>(m_entity);
} /* cache() */

NS_END(representation, fordyca);
