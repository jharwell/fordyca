/**
 * @file stateful_controller_repository.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/params/depth0/stateful_controller_repository.hpp"
#include "fordyca/params/occupancy_grid_parser.hpp"
#include "fordyca/params/block_selection_matrix_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_controller_repository::stateful_controller_repository(void) {
  register_parser<occupancy_grid_parser, occupancy_grid_params>(
      occupancy_grid_parser::kXMLRoot, occupancy_grid_parser::kHeader1);
  register_parser<block_selection_matrix_parser,
                  block_selection_matrix_params>(block_selection_matrix_parser::kXMLRoot,
                                                 block_selection_matrix_parser::kHeader1);
}

NS_END(depth0, params, fordyca);
