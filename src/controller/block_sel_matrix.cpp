/**
 * @file block_sel_matrix.cpp
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
#include "fordyca/controller/block_sel_matrix.hpp"
#include "fordyca/params/block_sel_matrix_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Class Constants
 ******************************************************************************/
constexpr char block_sel_matrix::kNestLoc[];
constexpr char block_sel_matrix::kCubePriority[];
constexpr char block_sel_matrix::kRampPriority[];
constexpr char block_sel_matrix::kSelExceptions[];

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_sel_matrix::block_sel_matrix(
    const struct params::block_sel_matrix_params* params) {
  this->insert(std::make_pair(kNestLoc, params->nest));
  this->insert(std::make_pair(kCubePriority, params->priorities.cube));
  this->insert(std::make_pair(kRampPriority, params->priorities.ramp));
  this->insert(std::make_pair(kSelExceptions, std::vector<int>()));
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_sel_matrix::sel_exception_add(int id) {
  boost::get<std::vector<int>>(this->find(kSelExceptions)->second).push_back(id);
} /* sel_exception_add() */

void block_sel_matrix::sel_exceptions_clear(void) {
  boost::get<std::vector<int>>(this->operator[](kSelExceptions)).clear();
} /* sel_exceptions_clear() */

NS_END(controller, fordyca);
