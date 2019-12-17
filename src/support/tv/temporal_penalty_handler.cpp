/**
 * \file temporal_penalty_handler.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/support/tv/temporal_penalty_handler.hpp"

#include "fordyca/controller/base_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void temporal_penalty_handler::penalty_abort(
    const controller::base_controller& controller) {
  std::scoped_lock lock(m_list_mtx);
  auto it = penalty_find(controller, false);
  if (m_penalty_list.end() != it) {
    penalty_remove(*it, false);
  }
  ER_INFO("%s", controller.GetId().c_str());
  ER_ASSERT(!is_serving_penalty(controller, false),
            "Robot still serving penalty after abort?!");
} /* penalty_abort() */

NS_END(tv, support, fordyca);
