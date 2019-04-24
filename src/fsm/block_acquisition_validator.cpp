/**
 * @file block_acquisition_validator.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/fsm/block_acquisition_validator.hpp"
#include "fordyca/ds/dp_block_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_acquisition_validator::block_acquisition_validator(
    const ds::dp_block_map* map)
    : ER_CLIENT_INIT("fordyca.fsm.block_acquisition_validator"), mc_map(map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_pure bool block_acquisition_validator::operator()(
    __rcsw_unused const rmath::vector2d& loc,
    uint id) const {
  auto block = mc_map->find(id);
  if (nullptr == block) {
    ER_WARN("Acquisition of free block%d@%s invalid: no such block",
            id,
            loc.to_str().c_str());
    return false;
  }
  return true;
} /* operator()() */

NS_END(fsm, fordyca);
