/**
 * @file cache_list.hpp
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

#ifndef INCLUDE_FORDYCA_DS_CACHE_LIST_HPP_
#define INCLUDE_FORDYCA_DS_CACHE_LIST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace repr {
class base_cache;
}
NS_START(ds);

using cache_list_type = std::shared_ptr<repr::base_cache>;
using const_cache_list_type = std::shared_ptr<const repr::base_cache>;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * @class cache_list
 * @ingroup ds
 *
 * @brief Specialization of @ref std::list with an additional @ref to_str()
 * method.
 */
class cache_list : public std::list<cache_list_type> {
 public:
  using std::list<cache_list_type>::list;
  using value_type = std::list<cache_list_type>::value_type;

  /**
   * @brief Get a string representation of the list contents.
   */
  std::string to_str(void) const;
};

/**
 * @class const_cache_list
 * @ingroup ds
 *
 * @brief Specialization of @ref std::list with an additional @ref to_str()
 * method.
 */
class const_cache_list : public std::list<const_cache_list_type> {
 public:
  using std::list<const_cache_list_type>::list;
  using value_type = std::list<const_cache_list_type>::value_type;

  /**
   * @brief Get a string representation of the list contents.
   */
  std::string to_str(void) const;
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_CACHE_LIST_HPP_ */
