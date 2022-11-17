/**
 * \file dp_cache_map.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/er/stringizable.hpp"

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
                                    carepr::base_cache>,
                     public rer::stringizable {
 public:
  using dpo_map<rmath::vector2z,
                carepr::base_cache>::dpo_map;

  std::string to_str(void) const override;
};

NS_END(ds, perception, subsystem, fordyca);

