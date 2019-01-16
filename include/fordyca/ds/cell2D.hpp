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

#ifndef INCLUDE_FORDYCA_DS_CELL2D_HPP_
#define INCLUDE_FORDYCA_DS_CELL2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/fsm/cell2D_fsm.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace representation {
class base_cache;
class base_block;
class base_cell_entity;
} // namespace representation

NS_START(ds);

namespace visitor = rcppsw::patterns::visitor;
namespace decorator = rcppsw::patterns::decorator;
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cell2D
 * @ingroup ds
 *
 * @brief Base representation of a cell on a 2D grid. A combination of FSM +
 * handle to whatever \ref cell_entity the cell contains, if any.
 */
class cell2D : public visitor::visitable_any<cell2D>,
               public decorator::decorator<fsm::cell2D_fsm> {
 public:
  cell2D(void);

  cell2D(const cell2D& other) = default;
  cell2D& operator=(const cell2D& other) = delete;

  bool operator==(const cell2D& other) const { return other.loc() == m_loc; }
  void robot_id(const std::string& robot_id) { m_robot_id = robot_id; }
  const std::string& robot_id(void) const { return m_robot_id; }

  fsm::cell2D_fsm& fsm(void) { return decoratee(); }
  const fsm::cell2D_fsm& fsm(void) const { return decoratee(); }

  /* state inquiry */
  RCPPSW_DECORATE_FUNC(state_is_known, const);
  RCPPSW_DECORATE_FUNC(state_has_block, const);
  RCPPSW_DECORATE_FUNC(state_has_cache, const);
  RCPPSW_DECORATE_FUNC(state_in_cache_extent, const);
  RCPPSW_DECORATE_FUNC(state_is_empty, const);

  /**
   * @brief Reset the cell to its UNKNOWN state.
   */
  void reset(void) {
    decoratee().init();
    m_entity.reset();
  }

  RCPPSW_DECORATE_FUNC(block_count, const);

  /**
   * @brief Set the entity associated with this cell.
   */
  void entity(const std::shared_ptr<representation::base_cell_entity>& entity) {
    m_entity = entity;
  }
  const std::shared_ptr<representation::base_cell_entity>& entity(void) const {
    return m_entity;
  }

  void loc(rmath::vector2u loc) { m_loc = loc; }
  rmath::vector2u loc(void) const { return m_loc; }

  /**
   * @brief Get the block entity associated with this cell.
   *
   * Will be NULL unless it contains a block, so check the cell's state before
   * calling this function.
   */
  std::shared_ptr<representation::base_block> block(void) const;
  std::shared_ptr<representation::base_block> block(void);

  /**
   * @brief Get the cache entity associated with this cell.
   *
   * Will be NULL unless it contains a cache, so check the cell's state before
   * calling this function.
   */
  std::shared_ptr<representation::base_cache> cache(void) const;
  std::shared_ptr<representation::base_cache> cache(void);

 private:
  /* clang-format off */
  std::string                                       m_robot_id{""};
  std::shared_ptr<representation::base_cell_entity> m_entity{nullptr};
  rmath::vector2u                             m_loc;
  /* clang-format on */
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_CELL2D_HPP_ */
