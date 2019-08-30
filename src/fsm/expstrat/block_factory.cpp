/**
 * @file block_factory.cpp
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
#include "fordyca/fsm/expstrat/block_factory.hpp"

#include "fordyca/fsm/expstrat/crw.hpp"
#include "fordyca/fsm/expstrat/likelihood_block_search.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_factory::block_factory(void) {
  register_type<crw>(kCRW);
  register_type<likelihood_block_search>(kLikelihoodSearch);
}

NS_END(expstrat, fsm, fordyca);
