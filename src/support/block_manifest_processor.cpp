/**
 * @file block_manifest_processor.cpp
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
#include "fordyca/support/block_manifest_processor.hpp"
#include "fordyca/representation/cube_block.hpp"
#include "fordyca/representation/ramp_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_manifest_processor::block_manifest_processor(
    const params::arena::block_manifest* const m)
    : mc_manifest(*m) {
  register_type<representation::cube_block>("cube");
  register_type<representation::ramp_block>("ramp");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ds::block_vector block_manifest_processor::create_blocks(void) {
  ds::block_vector v;
  uint i;
  for (i = 0; i < mc_manifest.n_cube; ++i) {
    v.push_back(create(
        "cube", rmath::vector2d(mc_manifest.unit_dim, mc_manifest.unit_dim), i));
  } /* for(i..) */
  for (i = mc_manifest.n_cube; i < mc_manifest.n_cube + mc_manifest.n_ramp;
       ++i) {
    v.push_back(
        create("ramp",
               rmath::vector2d(mc_manifest.unit_dim * 2, mc_manifest.unit_dim),
               i));
  } /* for(i..) */
  return v;
} /* create_blocks() */

NS_END(support, fordyca);
