/**
 * \file block_transporter.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

#include "fordyca/fsm/foraging_transport_goal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_transporter
 * \ingroup fsm
 *
 * \brief Interface defining what classes directly involved in transporting
 * blocks need to implement in order to successfully interact with the loop
 * functions.
 */
class block_transporter {
 public:
  block_transporter(void) = default;
  virtual ~block_transporter(void) = default;

  /**
   * \brief All tasks must define method to determine what they are currently
   * doing with a block (if they are carrying one).
   */
  virtual foraging_transport_goal block_transport_goal(void) const = 0;
};

NS_END(fsm, fordyca);
