/**
 * @file block_carry_visualizer.hpp
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
#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_CARRY_VISUALIZER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_CARRY_VISUALIZER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace argos {
class CQTOpenGLUserFunctions;
}
NS_START(fordyca);
namespace representation {
class base_block;
}
NS_START(support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class block_carry_visualizer
 * @ingroup support
 *
 * @brief Renders a block in 3D that the robot is carrying for
 * visualization/debugging purposes.
 */
class block_carry_visualizer
    : public rcppsw::er::client<block_carry_visualizer> {
 public:
  block_carry_visualizer(argos::CQTOpenGLUserFunctions* qt,
                         double block_vis_offset,
                         double text_vis_offset)
      : ER_CLIENT_INIT("fordyca.support.block_carry_visualizer"),
        m_block_vis_offset(block_vis_offset),
        m_text_vis_offset(text_vis_offset),
        m_qt(qt) {}

  block_carry_visualizer(const block_carry_visualizer& op) = delete;
  block_carry_visualizer& operator=(const block_carry_visualizer& op) = delete;

  /**
   * @brief Draw visualizations related to block carries:
   *
   * - The block itself
   * - The block ID
   *
   * @param block The block to draw.
   * @param id_len The length of the robot ID. Used to ensure block ID does not
   * overlap with robot ID, if both visualizations are enabled.
   */
  void draw(const representation::base_block* block, uint robot_id);

  /* clang-format off */
  double                               m_block_vis_offset{0.0};
  double                               m_text_vis_offset{0.0};
  argos::CQTOpenGLUserFunctions* const m_qt{nullptr};
  /* clang-format on */
};

NS_END(fordyca, support);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_CARRY_VISUALIZER_HPP_ */
