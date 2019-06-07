/**
 * @file unicell_entity.hpp
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

#ifndef INCLUDE_FORDYCA_REPR_UNICELL_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPR_UNICELL_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/repr/multicell_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class unicell_entity
 * @ingroup fordyca repr
 *
 * @brief Representation of an entity in the arena that only spans a single
 * cell.
 */
using unicell_entity = multicell_entity;

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_UNICELL_ENTITY_HPP_ */
