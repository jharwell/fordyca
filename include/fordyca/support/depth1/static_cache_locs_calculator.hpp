/**
 * \file static_cache_locs_calculator.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_LOCS_CALCULATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_LOCS_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/foraging/config/block_dist_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class static_cache_locs_calculator
 * \ingroup support depth1
 *
 * \brief Calculates the locations of static caches in the arena during
 * initialization, based on the configured block distribution.
 *
 * For the single source, dual source, quad source block distributions, each of
 * the static caches is halfway between the center of the nest and a block
 * cluster. Arena map initialization has already happened at this point, as the
 * created block clusters are needed.
 *
 * For the powerlaw distribution, arena map initialization has not happened yet,
 * because we need the clusters (which are randomly positioned in the arena) to
 * avoid the locations of the static caches. We can't know in advance where
 * these locations will be so we have to calculate cache locations BEFORE
 * initializing the arena map.
 */
class static_cache_locs_calculator : public rer::client<static_cache_locs_calculator> {
 public:
  static_cache_locs_calculator(void)
      : ER_CLIENT_INIT("fordyca.support.depth1.static_cache_locs_calculator") {}

  /* Not move/copy constructable/assignable by default */
  static_cache_locs_calculator(const static_cache_locs_calculator&) = delete;
  const static_cache_locs_calculator& operator=(const static_cache_locs_calculator&) = delete;
  static_cache_locs_calculator(static_cache_locs_calculator&&) = delete;
  static_cache_locs_calculator& operator=(static_cache_locs_calculator&&) = delete;


  std::vector<rmath::vector2d> operator()(
      const carena::caching_arena_map* arena_map,
      const cfconfig::block_dist_config* distp) RCSW_COLD;
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_LOCS_CALCULATOR_HPP_ */
