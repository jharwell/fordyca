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
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace representation {
class base_cache;
struct perceived_cache;
} // namespace representation
NS_START(ds);

using cache_list_type = std::shared_ptr<representation::base_cache>;
using const_cache_list_type = std::shared_ptr<const representation::base_cache>;
using perceived_cache_list_type = representation::perceived_cache;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
using cache_list = std::list<cache_list_type>;
using const_cache_list = std::list<const_cache_list_type>;
using perceived_cache_list = std::list<perceived_cache_list_type>;

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_CACHE_LIST_HPP_ */
