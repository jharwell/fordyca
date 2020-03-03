/**
 * \file dpo_semantic_map.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_DS_DPO_SEMANTIC_MAP_HPP_
#define INCLUDE_FORDYCA_DS_DPO_SEMANTIC_MAP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "rcppsw/patterns/decorator/decorator.hpp"

#include "fordyca/ds/dp_block_map.hpp"
#include "fordyca/ds/dp_cache_map.hpp"
#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/ds/occupancy_grid.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);
namespace config { namespace perception {
struct perception_config;
}} // namespace config::perception

NS_START(ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_semantic_map
 * \ingroup ds
 *
 * \brief Stores a semantic map of the state of the arena, from the perspective
 * of the robot (i.e. th physical extent of the arena + semantic information
 * about what is in it/its characteristics such as likelihood of
 * collision).
 *
 * Contains:
 *
 * - A mapped extent divided into identical cells (\ref occupancy_grid).
 * - A set of objects in that extent (\ref dpo_store).
 *
 * Does *NOT* track which cells are in CACHE_EXTENT, as that is irrelevant for
 * what the robots need (as of 9/14/18 anyway).
 */
class dpo_semantic_map final : public rer::client<dpo_semantic_map>,
                               public rpdecorator::decorator<occupancy_grid> {
 public:
  dpo_semantic_map(const config::perception::perception_config* c_config,
                   const std::string& robot_id);

  RCPPSW_DECORATE_FUNC(pheromone_repeat_deposit, const);

  /**
   * \brief Access a particular element in the discretized grid representing the
   * robot's view of the arena. No bounds checking is performed, so if something
   * is out of bounds, boost will fail with a bounds checking assertion.
   *
   * \param i X coord.
   * \param j Y coord
   *
   * \return The cell.
   */
  template <size_t Index>
  typename occupancy_grid::layer_value_type<Index>::value_type& access(size_t i,
                                                                       size_t j) {
    return decoratee().access<Index>(i, j);
  }
  template <size_t Index>
  const typename occupancy_grid::layer_value_type<Index>::value_type& access(
      size_t i,
      size_t j) const {
    return decoratee().access<Index>(i, j);
  }
  template <size_t Index>
  typename occupancy_grid::layer_value_type<Index>::value_type& access(
      const rmath::vector2u& d) {
    return decoratee().access<Index>(d);
  }
  template <size_t Index>
  const typename occupancy_grid::layer_value_type<Index>::value_type& access(
      const rmath::vector2u& d) const {
    return decoratee().access<Index>(d);
  }

  /**
   * \brief Update the density of:
   *
   * - All cells in the arena.
   * - All known objects in the arena.
   */
  void decay_all(void);

  /**
   * \brief Reset all the cells in the percieved arena.
   */
  RCPPSW_DECORATE_FUNC(reset)
  RCPPSW_DECORATE_FUNC(xdsize, const)
  RCPPSW_DECORATE_FUNC(ydsize, const)
  RCPPSW_DECORATE_FUNC(xrsize, const)
  RCPPSW_DECORATE_FUNC(yrsize, const)
  RCPPSW_DECORATE_FUNC(known_cells_inc)
  RCPPSW_DECORATE_FUNC(known_cells_dec)
  RCPPSW_DECORATE_FUNC(known_cell_count, const)
  RCPPSW_DECORATE_FUNC(resolution, const)

  bool cache_remove(cfrepr::base_cache* victim);
  bool block_remove(crepr::base_block2D* victim);

  const dpo_store* store(void) const { return &m_store; }
  dpo_store* store(void) { return &m_store; }

 private:
  /* clang-format off */
  dpo_store m_store;
  /* clang-format on */

 public:
  /* wrapping DPO store--must be after declaration -_- */
  RCPPSW_DECLDEF_WRAP(blocks, (*store()))
  RCPPSW_DECLDEF_WRAP(caches, (*store()))
  RCPPSW_DECLDEF_WRAP(blocks, (*store()), const)
  RCPPSW_DECLDEF_WRAP(caches, (*store()), const)
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_DPO_SEMANTIC_MAP_HPP_ */
