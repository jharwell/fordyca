/**
 * \file entity_list.hpp
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

#ifndef INCLUDE_FORDYCA_DS_ENTITY_LIST_HPP_
#define INCLUDE_FORDYCA_DS_ENTITY_LIST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class unicell_entity2D;
} // namespace cosm::repr

NS_START(fordyca, ds);

using entity_list_type = crepr::unicell_entity2D*;
using const_entity_list_type = const crepr::unicell_entity2D*;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
using entity_list = std::list<entity_list_type>;
using const_entity_list = std::list<const_entity_list_type>;

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_ENTITY_LIST_HPP_ */
