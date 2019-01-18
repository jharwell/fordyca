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
#include "fordyca/support/depth0/controller_interactor_mapper.hpp"
#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/support/depth0/robot_arena_interactor.hpp"

/*******************************************************************************
 * Namespaces/decls
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void controller_interactor_mapper::operator()(crw_itype& interactor) const {
  interactor(*dynamic_cast<crw_itype::controller_type*>(m_controller),
             m_timestep);
}
void controller_interactor_mapper::operator()(dpo_itype& interactor) const {
  interactor(*dynamic_cast<dpo_itype::controller_type*>(m_controller),
             m_timestep);
}
void controller_interactor_mapper::operator()(mdpo_itype& interactor) const {
  interactor(*dynamic_cast<mdpo_itype::controller_type*>(m_controller),
             m_timestep);
}

NS_END(depth0, support, fordyca);
