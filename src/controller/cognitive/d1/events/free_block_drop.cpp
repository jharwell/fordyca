/**
 * \file free_block_drop.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d2/events/free_block_drop.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_drop::free_block_drop(std::unique_ptr<crepr::base_block3D> block,
                                 const rmath::vector2z& coord,
                                 const rtypes::discretize_ratio& resolution)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d1.events.free_block_drop"),
      cell2D_op(coord),
      mc_resolution(resolution),
      m_block(std::move(block)) {}

NS_END(events, d1, cognitive, controller, fordyca);
