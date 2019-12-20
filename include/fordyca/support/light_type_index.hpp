/**
 * \file light_type_index.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_LIGHT_TYPE_INDEX_HPP_
#define INCLUDE_FORDYCA_SUPPORT_LIGHT_TYPE_INDEX_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>

#include "rcppsw/utils/color.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class light_type_index
 * \ingroup support
 *
 * \brief Index mapping an entity type to the color of the light that should be
 * associated with it, so that what things should have lights of what color is
 * not hardcoded in multiple places in the code, and is instead centralized
 * here. For usage by both robots and loop functions.
 *
 * Currently maps:
 *
 * - \ref repr::nest
 * - \ref repr::arena_cache
 */

class light_type_index {
 public:
  static constexpr char kNest[] = "nest";
  static constexpr char kCache[] = "cache";

  light_type_index(void);

  const rutils::color& operator[](const std::string& key) {
    return m_index[key];
  }

 private:
  /* clang-format off */
  std::map<std::string, rutils::color> m_index;
  /* clang-format on */
};

NS_END(fordyca, support);

#endif /* INCLUDE_FORDYCA_SUPPORT_LIGHT_TYPE_INDEX_HPP_ */
