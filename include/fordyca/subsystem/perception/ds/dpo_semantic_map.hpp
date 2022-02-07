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

#ifndef INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DS_DPO_SEMANTIC_MAP_HPP_
#define INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DS_DPO_SEMANTIC_MAP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "rcppsw/patterns/decorator/decorator.hpp"

#include "fordyca/subsystem/perception/config/mdpo_config.hpp"
#include "fordyca/subsystem/perception/ds/dp_block_map.hpp"
#include "fordyca/subsystem/perception/ds/dp_cache_map.hpp"
#include "fordyca/subsystem/perception/ds/dpo_store.hpp"
#include "fordyca/subsystem/perception/ds/occupancy_grid.hpp"
#include "fordyca/subsystem/perception/foraging_memory_model.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_semantic_map
 * \ingroup subsystem perception ds
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
                               public rpdecorator::decorator<occupancy_grid>,
                               public foraging_memory_model<ds::dp_block_map,
                                                                ds::dp_cache_map> {
 public:
  explicit dpo_semantic_map(const config::mdpo_config* c_config);

  const dpo_store* store(void) const { return &m_store; }
  dpo_store* store(void) { return &m_store; }

  /* access_known_blocks overrides */
  RCPPSW_WRAP_DECLDEF_OVERRIDE(known_blocks, (*store()), const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(known_caches, (*store()), const)

  /* foraging_memory_model overrides */
  bool cache_remove(carepr::base_cache* victim) override;
  bool block_remove(crepr::sim_block3D* victim) override;
  model_update_result block_update(tracked_block_type&& block) override;
  model_update_result cache_update(tracked_cache_type&& cache) override;

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
  const typename occupancy_grid::layer_value_type<Index>::value_type&
  access(size_t i, size_t j) const {
    return decoratee().access<Index>(i, j);
  }
  template <size_t Index>
  typename occupancy_grid::layer_value_type<Index>::value_type&
  access(const rmath::vector2z& d) {
    return decoratee().access<Index>(d);
  }
  template <size_t Index>
  const typename occupancy_grid::layer_value_type<Index>::value_type&
  access(const rmath::vector2z& d) const {
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
  RCPPSW_DECORATE_DECLDEF(reset)
  RCPPSW_DECORATE_DECLDEF(xdsize, const)
  RCPPSW_DECORATE_DECLDEF(ydsize, const)
  RCPPSW_DECORATE_DECLDEF(xrsize, const)
  RCPPSW_DECORATE_DECLDEF(yrsize, const)
  RCPPSW_DECORATE_DECLDEF(known_cells_inc)
  RCPPSW_DECORATE_DECLDEF(known_cells_dec)
  RCPPSW_DECORATE_DECLDEF(known_cell_count, const)
  RCPPSW_DECORATE_DECLDEF(resolution, const)

  RCPPSW_DECORATE_DECLDEF(pheromone_repeat_deposit, const);

 private:
  /* clang-format off */
  dpo_store m_store;
  /* clang-format on */
};

NS_END(ds, perception, subsystem, fordyca);

#endif /* INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DS_DPO_SEMANTIC_MAP_HPP_ */
