/**
 * @file controller_interactor_mapper.cpp
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
/*
 * This is needed because without it boost instantiates static assertions that
 * verify that every possible handler<controller> instantiation is valid, which
 * includes checking for depth1 controllers being valid for new cache drop/cache
 * site drop events. These will not happen in reality (or shouldn't), and if
 * they do it's 100% OK to crash with an exception.
 */
#define BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT

#include "fordyca/support/depth2/controller_interactor_mapper.hpp"
#include "fordyca/controller/depth2/grp_dpo_controller.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/support/depth2/robot_arena_interactor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool controller_interactor_mapper::operator()(grp_dpo_itype& interactor) const {
  return interactor(*dynamic_cast<grp_dpo_itype::controller_type*>(controller()),
                    timestep());
}
bool controller_interactor_mapper::operator()(grp_mdpo_itype& interactor) const {
  return interactor(
      *dynamic_cast<grp_mdpo_itype::controller_type*>(controller()), timestep());
}

NS_END(depth2, support, fordyca);
