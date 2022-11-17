/**
 * \file block_sel_matrix.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
  std::get<std::vector<rtypes::type_uuid>>(this->find(kSelExceptions)->second)
      .push_back(id);
} /* sel_exception_add() */

void block_sel_matrix::sel_exceptions_clear(void) {
  std::get<std::vector<rtypes::type_uuid>>(this->operator[](kSelExceptions))
      .clear();
} /* sel_exceptions_clear() */

NS_END(cognitive, controller, fordyca);
