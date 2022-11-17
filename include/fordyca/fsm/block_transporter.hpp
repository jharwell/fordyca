/**
 * \file block_transporter.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
