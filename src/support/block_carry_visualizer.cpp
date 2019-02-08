/**
 * @file block_carry_visualizer.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/support/block_carry_visualizer.hpp"

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/cube_block.hpp"
#include "fordyca/representation/ramp_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_carry_visualizer::draw(const representation::base_block* const block,
                                  uint id_len) {
  if (nullptr != dynamic_cast<const representation::cube_block*>(block)) {
    m_qt->DrawBox(argos::CVector3(0.0, 0.0, m_block_vis_offset),
                  argos::CQuaternion(),
                  argos::CVector3(block->xsize(), block->xsize(), block->xsize()),
                  argos::CColor(block->color().red(),
                                block->color().green(),
                                block->color().blue()));
  } else if (nullptr != dynamic_cast<const representation::ramp_block*>(block)) {
    /*
     * Ramp blocks are 2X as long in X as in Y. Z height is the same as X (not
     * currently used/handled in the simulation; only for visualization
     * purposes). But, drawing ramp blocks in ARGos is a pain in the ass, so use
     * the spherical cow approach.
     */
    m_qt->DrawBox(argos::CVector3(0.0, 0.0, m_block_vis_offset),
                  argos::CQuaternion(),
                  argos::CVector3(block->xsize(), block->ysize(), block->ysize()),
                  argos::CColor(block->color().red(),
                                block->color().green(),
                                block->color().blue()));

  } else {
    /* Unknown block type */
    ER_FATAL_SENTINEL("Cannot visualize unknown block type: Not cube or ramp");
  }
  if (block->display_id()) {
    m_qt->DrawText(argos::CVector3(0.0, 0.0, m_text_vis_offset),
                   std::string(id_len + 3, ' ') + "[b" +
                       std::to_string(block->id()) + "]",
                   argos::CColor::GREEN);
  }
}

NS_END(support, fordyca);
