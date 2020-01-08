/**
 * \file dp_cache_map.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/ds/dp_cache_map.hpp"

#include <numeric>

#include "fordyca/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string dp_cache_map::to_str(void) const {
  auto range = const_values_range();
  return std::accumulate(range.begin(),
                         range.end(),
                         std::string(),
                         [&](const std::string& a, const auto& pair) {
                           return a + "c" + rcppsw::to_string(pair.ent()->id()) +
                                  "@" + pair.ent()->dloc().to_str() + ",";
                         });
} /* to_str() */

NS_END(ds, fordyca);
