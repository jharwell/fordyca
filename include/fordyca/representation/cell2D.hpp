/**
 * @file cell2D.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_CELL2D_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_CELL2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/common/er_server.hpp"
#include "fordyca/fsm/cell2D_fsm.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

namespace visitor = rcppsw::patterns::visitor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Base representation of a cell on a 2D grid. This class represents
 * the ACTUAL state of the grid (i.e. not whatever robots happen to think the
 * state is).
 */
class cell2D : public visitor::visitable_any<cell2D> {
 public:
  explicit cell2D(const std::shared_ptr<rcppsw::common::er_server>& server);

  /* state inquiry */

  /**
   * @brief If TRUE, the state is currently KNOWN.
   */
  bool state_is_known(void) const { return m_fsm.state_is_known(); }

  /**
   * @brief If TRUE, the state is currently known to contain a block.
   */
  bool state_has_block(void) const { return m_fsm.state_has_block(); }

  /**
   * @brief If TRUE, the state is currently known to contain a cache.
   */
  bool state_has_cache(void) const { return m_fsm.state_has_cache(); }

  /**
   * @brief If TRUE, the state is currently known to be empty.
   */
  bool state_is_empty(void) const { return m_fsm.state_is_empty(); }

  /**
   * @brief Reset the cell to its UNKNOWN state.
   */
  void reset(void) { m_fsm.init(); }

  /**
   * @brief Set the entity associated with this cell.
   */
  void entity(representation::cell_entity* entity) { m_entity = entity; }
  void loc(discrete_coord loc) { m_loc = loc; }
  discrete_coord loc(void) const { return m_loc; }

  /**
   * @brief Get the block entity associated with this cell.
   *
   * Will be NULL unless it contains a block, so check the cell's state before
   * calling this function.
   */
  const representation::block* block(void) const {
    return static_cast<representation::block*>(m_entity);
  }

  /**
   * @brief Get the cache entity associated with this cell.
   *
   * Will be NULL unless it contains a block, so check the cell's state before
   * calling this function.
   */
  const representation::cache* cache(void) const {
    return static_cast<representation::cache*>(m_entity);
  }

  fsm::cell2D_fsm& fsm(void) { return m_fsm; }

 private:
  cell2D(const cell2D& other) = delete;
  cell2D& operator=(const cell2D& other) = delete;

  cell_entity* m_entity;
  discrete_coord m_loc;
  fsm::cell2D_fsm m_fsm;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_CELL2D_HPP_ */
