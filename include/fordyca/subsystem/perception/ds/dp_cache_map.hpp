/**
 * \file dp_cache_map.hpp
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

#ifndef INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DS_DP_CACHE_MAP_HPP_
#define INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DS_DP_CACHE_MAP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/math/vector2.hpp"

#include "fordyca/subsystem/perception/ds/dpo_map.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {
class base_cache;
}

NS_START(fordyca, subsystem, perception, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dp_cache_map
 * \ingroup subsystem perception ds
 *
 * \brief The cache map is a repr of the robot's perception of caches
 * in the arena. It uses the locations of caches as keys, as caches are
 * immovable during simulation. Using cache IDs as keys for insertion/removal
 * would result in incorrect behavior, as a cache with ID 0 that has been
 * depleted would not be replaced with a newer version of that cache with ID 1
 * during LOS process (it would be inserted into the map, but the old version
 * would not be removed, as they would be considered different objects).
 */
class dp_cache_map : public dpo_map<rmath::vector2z,
                                    carepr::base_cache> {
 public:
  using dpo_map<rmath::vector2z,
                carepr::base_cache>::dpo_map;

  /**
   * \brief Build a string from the list of DP caches that a robot is tracking
   * for logging.
   */
  std::string to_str(void) const;
};

NS_END(ds, perception, subsystem, fordyca);

#endif /* INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DS_DP_CACHE_MAP_HPP_ */
