/**
 * \file d1_qt_user_functions.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/d0/d0_qt_user_functions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, d1);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class d1_qt_user_functions
 * \ingroup argos support d1
 *
 * \brief Contains hooks for Qt to draw the visualizations related to depth 1
 * task decomposition:
 *
 * - Task name
 */
class d1_qt_user_functions : public d0::d0_qt_user_functions {
 public:
  d1_qt_user_functions(void);
  ~d1_qt_user_functions(void) override = default;

  void Draw(chal::robot& c_entity);
};

NS_END(d1, support, argos, fordyca);

