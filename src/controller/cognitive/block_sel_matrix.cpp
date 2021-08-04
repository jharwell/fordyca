/**
 * \file block_sel_matrix.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/block_sel_matrix.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_sel_matrix::block_sel_matrix(
    const config::block_sel::block_sel_matrix_config* config,
    const rmath::vector2d& nest_loc) {
  this->insert(std::make_pair(kNestLoc, nest_loc));
  this->insert(std::make_pair(kCubePriority, config->priorities.cube));
  this->insert(std::make_pair(kRampPriority, config->priorities.ramp));
  this->insert(std::make_pair(kSelExceptions, std::vector<rtypes::type_uuid>()));
  this->insert(std::make_pair(kPickupPolicy, config->pickup_policy));
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_sel_matrix::sel_exception_add(const rtypes::type_uuid& id) {
  boost::get<std::vector<rtypes::type_uuid>>(this->find(kSelExceptions)->second)
      .push_back(id);
} /* sel_exception_add() */

void block_sel_matrix::sel_exceptions_clear(void) {
  boost::get<std::vector<rtypes::type_uuid>>(this->operator[](kSelExceptions))
      .clear();
} /* sel_exceptions_clear() */

NS_END(cognitive, controller, fordyca);
