/**
 * \file dp_cache_map.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/subsystem/perception/ds/dp_cache_map.hpp"

#include <numeric>

#include "cosm/arena/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, ds);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string dp_cache_map::to_str(void) const {
  auto range = values_range();
  return std::accumulate(range.begin(),
                         range.end(),
                         std::string(),
                         [&](const std::string& a, const auto& pair) {
                           return a + "c" + rcppsw::to_string(pair.ent()->id()) +
                                  "@" +
                                  rcppsw::to_string(pair.ent()->dcenter2D()) +
                                  ",";
                         });
} /* to_str() */

NS_END(ds, perception, subsystem, fordyca);
