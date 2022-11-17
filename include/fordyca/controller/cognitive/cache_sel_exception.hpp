/**
 * \file cache_sel_exception.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/type_uuid.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive);

/*******************************************************************************
 * Struct Definitions
******************************************************************************/
/**
 * \class cache_sel_exception
 * \ingroup controller cognitive
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

NS_END(cognitive, controller, fordyca);

