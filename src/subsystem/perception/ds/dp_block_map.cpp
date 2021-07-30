/**
 * \file dp_block_map.cpp
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
#include "fordyca/subsystem/perception/ds/dp_block_map.hpp"

#include <numeric>

#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, ds);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string dp_block_map::to_str(void) const {
  auto range = values_range();
  return std::accumulate(range.begin(),
                         range.end(),
                         std::string(),
                         [&](const std::string& a, const auto& b) {
                           return a + "b" + rcppsw::to_string(b.ent()->id()) +
                                  ",";
                         });
} /* to_str() */

NS_END(ds, perception, subsystem, fordyca);
