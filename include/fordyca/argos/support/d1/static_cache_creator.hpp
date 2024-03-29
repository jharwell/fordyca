/**
 * \file static_cache_creator.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include <vector>

#include "rcppsw/types/spatial_dist.hpp"

#include "fordyca/argos/support/caches/base_creator.hpp"
#include "fordyca/ds/block_alloc_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, d1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class static_cache_creator
 * \ingroup argos support d1
 *
 * \brief Creates static cache(s) in the arena by taking \ref
 * base_cache::kMinBlocks from the allocated blocks and grouping them together
 * at each of the specified cache locations where a cache does not currently
 * exist in order to create a new static cache.
 */
class static_cache_creator : public fascaches::base_creator,
                             public rer::client<static_cache_creator> {
 public:
  static_cache_creator(carena::caching_arena_map* map,
                       const std::vector<rmath::vector2d>& centers,
                       const rtypes::spatial_dist& cache_dim);
  ~static_cache_creator(void) override = default;

  /**
   * \brief Re-create all static caches. Ignores block cluster locations because
   * the locations of the static caches do not change and are known
   * (err...assumed) to be conflict free.
   *
   * \param c_params Cache creation parameters
   * \param c_alloc_map The blocks which have been allocated for the creation
   *                     of ALL static caches this timestep.
   */
  creation_result create_all(const fascaches::create_ro_params& c_params,
                             ds::block_alloc_map&& c_alloc_map,
                             bool pre_dist);

 private:
  /* clang-format off */
  const std::vector<rmath::vector2d> mc_centers;
  /* clang-format on */
};

NS_END(d1, support, argos, fordyca);
