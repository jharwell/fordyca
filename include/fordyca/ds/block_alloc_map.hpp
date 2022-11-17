/**
 * \file block_alloc_map.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

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

  bool contains(const crepr::sim_block3D* block) const {
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
  RCPPSW_WRAP_DECLDEF(at, decoratee(), const);
  RCPPSW_WRAP_DECLDEF(operator[], decoratee());
  RCPPSW_WRAP_DECLDEF(begin, decoratee());
  RCPPSW_WRAP_DECLDEF(end, decoratee());
  RCPPSW_WRAP_DECLDEF(begin, decoratee(), const);
  RCPPSW_WRAP_DECLDEF(end, decoratee(), const);
};

NS_END(ds, fordyca);
