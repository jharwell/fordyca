/**
 * @file dbg.hpp
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

#ifndef INCLUDE_FORDYCA_DBG_DBG_HPP_
#define INCLUDE_FORDYCA_DBG_DBG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <numeric>
#include <string>

#include "fordyca/representation/base_cell_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, dbg);

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
template <typename T>
std::string blocks_list(const T& blocks) {
  std::string ret =
      std::accumulate(blocks.begin(),
                      blocks.end(),
                      std::string(),
                      [&](const std::string& a, const auto& b) {
                        return a + "b" + std::to_string(b->id()) + ",";
                      });
  return ret;
}

template <typename T>
std::string caches_list(const T& caches) {
  std::string ret =
      std::accumulate(caches.begin(),
                      caches.end(),
                      std::string(),
                      [&](const std::string& a, const auto& b) {
                        return a + "c" + std::to_string(b->id()) + ",";
                      });
  return ret;
}

NS_END(dbg, fordyca);

#endif /* INCLUDE_FORDYCA_DBG_DBG_HPP_ */
