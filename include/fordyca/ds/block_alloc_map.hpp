/**
 * \file block_alloc_map.hpp
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

#ifndef INCLUDE_FORDYCA_DS_BLOCK_ALLOC_MAP_HPP_
#define INCLUDE_FORDYCA_DS_BLOCK_ALLOC_MAP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <map>

#include "cosm/ds/block3D_vector.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_alloc_map
 * \ingroup ds
 *
 * \brief Wrapper around std::map: (cache ID, block alloc vector). For use in
 * cache creation.
 */

class block_alloc_map {
 public:
  using map_type = std::map<int, cds::block3D_vectorno>;
  block_alloc_map(void) = default;

  bool contains(const crepr::base_block3D* block) const {
    auto cache_i_alloc = [&](const auto& alloc_i) {
      return alloc_i.second.end() !=
             std::find(alloc_i.second.begin(), alloc_i.second.end(), block);
    };
    return std::any_of(m_decoratee.begin(), m_decoratee.end(), cache_i_alloc);
  }

 private:
  const map_type& decoratee(void) const { return m_decoratee; }
  map_type& decoratee(void) { return m_decoratee; }

  /* clang-format off */
  map_type m_decoratee{};
  /* clang-format on */

 public:
  RCPPSW_WRAP_DECLDEF(operator[], decoratee());
  RCPPSW_WRAP_DECLDEF(begin, decoratee());
  RCPPSW_WRAP_DECLDEF(end, decoratee());
  RCPPSW_WRAP_DECLDEF(begin, decoratee(), const);
  RCPPSW_WRAP_DECLDEF(end, decoratee(), const);
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_BLOCK_ALLOC_MAP_HPP_ */
