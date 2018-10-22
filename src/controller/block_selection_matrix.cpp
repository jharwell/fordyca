/**
 * @file block_selection_matrix.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/block_selection_matrix.hpp"
#include "fordyca/params/block_selection_matrix_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_selection_matrix::block_selection_matrix(
    const struct params::block_selection_matrix_params * params) {
  this->insert(std::make_pair("nest_loc", params->nest));
  this->insert(std::make_pair("cube_priority", params->priorities.cube));
  this->insert(std::make_pair("ramp_priority", params->priorities.ramp));
}

NS_END(controller, fordyca);
