/**
 * @file throttling_handler.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/controller/throttling_handler.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/params/throttling_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
throttling_handler::throttling_handler(
    const struct params::throttling_params * const params)
    : m_block_carry(params->block_carry) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void throttling_handler::update(void) {
  if (m_carrying_block) {
    m_block_current = m_block_carry;
  } else {
    m_block_current = 0;
  }
} /* update() */


NS_END(controller, fordyca);
