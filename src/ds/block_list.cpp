/**
 * \file block_list.cpp
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
#include "fordyca/ds/block_list.hpp"

#include <numeric>

#include "fordyca/repr/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
std::string block_list::to_str(void) const {
  return std::accumulate(this->begin(),
                         this->end(),
                         std::string(),
                         [&](const std::string& a, const auto& b) {
                           return a + "b" + std::to_string(b->id()) + ",";
                         });
} /* to_string() */

std::string const_block_list::to_str(void) const {
  return std::accumulate(this->begin(),
                         this->end(),
                         std::string(),
                         [&](const std::string& a, const auto& b) {
                           return a + "b" + std::to_string(b->id()) + ",";
                         });
} /* to_string() */

NS_END(ds, fordyca);
