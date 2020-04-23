/**
 * \file cache_sel_exception.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_CACHE_SEL_EXCEPTION_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_CACHE_SEL_EXCEPTION_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/type_uuid.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Struct Definitions
******************************************************************************/
/**
 * \class cache_sel_exception
 * \ingroup controller
 *
 * \brief Once an object has been dropped (picked up) in a cache, this structure
 * is used to disallow the robot from immediately using that same cache again
 * (which it may try to do, depending on what task it is doing).
 */
struct cache_sel_exception {
  enum type { ekPICKUP, ekDROP };

  rtypes::type_uuid id;
  enum type type;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_CACHE_SEL_EXCEPTION_HPP_ */
