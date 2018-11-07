/**
 * @file dynamic_cache_creator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_CREATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <random>
#include "fordyca/support/base_cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
namespace er = rcppsw::er;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class dynamic_cache_creator
 * @ingroup support depth2
 *
 * @brief Handles creation of dynamic caches during simulation, given a set of
 * candidate blocks, and constraints on proximity, minimum # for a cache, etc.
 */
class dynamic_cache_creator : public base_cache_creator,
                              public er::client<dynamic_cache_creator> {
 public:
  dynamic_cache_creator(ds::arena_grid* grid,
                        double cache_dim,
                        double min_dist,
                        uint min_blocks);

  /**
   * @brief Create new caches in the arena from blocks that are close enough
   * together.
   */
  cache_vector create_all(const cache_vector& existing_caches,
                          block_vector& candidate_blocks) override;

 private:
  static constexpr uint kOVERLAP_SEARCH_MAX_TRIES = 10;

  /**
   * @brief Calculate the center of the new cache that will be constructed from
   * the specified blocks.
   *
   * Ideally that will be just the average of the x and y coordinates of all the
   * constituent blocks. However, it is possible that placing a cache at that
   * location will cause it to overlap with other caches, and so corrections may
   * be necessary.
   *
   * @param blocks The list of blocks to create a new cache from.
   * @param existing_caches Vector of existing caches in the arena.
   *
   * @return Coordinates of the new cache.
   */
  argos::CVector2 calc_center(const block_list& blocks,
                              const cache_vector& existing_caches) const;

  /**
   * @brief Basic sanity checks on newly created caches.
   */
  bool creation_sanity_checks(const cache_vector& new_caches) const;

  /**
   * @brief Deconflict new cache cache from overlapping with any existing caches
   * (including taking the span of the new cache into account)
   *
   * @return \c TRUE if there were no conflicts, \c FALSE otherwise.
   */
  bool deconflict_cache_center(const representation::base_cache& cache,
                               argos::CVector2& center) const;
  // clang-format off
  double                             m_min_dist;
  uint                               m_min_blocks;
  mutable std::default_random_engine m_rng{};
  // clang-format on
};


NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_CREATOR_HPP_ */
