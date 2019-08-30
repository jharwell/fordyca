/**
 * @file dynamic_cache_config.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_CACHES_DYNAMIC_CACHE_CONFIG_HPP_
#define INCLUDE_FORDYCA_CONFIG_CACHES_DYNAMIC_CACHE_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/types/spatial_dist.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, caches);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct dynamic_cache_config
 * @ingroup fordyca config cache
 */
struct dynamic_cache_config : public rconfig::base_config {
  bool   enable{false};

  /**
   * @brief How close do blocks have to be to each other to be considered for
   * dynamic cache creation (should be >= whatever the threshold value for the
   * \ref cache_sel_matrix is, or weird behavior will likely result).
   */
  rtypes::spatial_dist min_dist{0.0};

  /**
   * @brief How many blocks within min_dist does it take to trigger dynamic
   * cache creation?
   */
  uint   min_blocks{0};

  /**
   * @brief If \c TRUE, then dynamic cache creation will only occur when a robot
   * drops a block (if the other conditions for dynamic cache creation are also
   * met of course). If \c FALSE, then it will occur ANYTIME the conditions for
   * dynamic cache creation are met (viz, min # blocks and block
   * proximity). This would allow cache creation from blocks clustered together
   * in single or dual source scenarios at the start of simulation, or as a
   * result of a block distribution after a robot drops a block in the nest,
   * which may not be desirable.
   */
  bool   robot_drop_only{false};
};

NS_END(caches, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_CACHES_DYNAMIC_CACHE_CONFIG_HPP_ */
