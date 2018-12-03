/**
 * @file los_visualizer.hpp
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
#ifndef INCLUDE_FORDYCA_SUPPORT_LOS_VISUALIZER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_LOS_VISUALIZER_HPP_

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
class line_of_sight;
}

NS_START(support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class los_visualizer
 * @ingroup support
 *
 * @brief Renders the LOS for a given robot for visualization/debugging
 * purposes.
 */
class los_visualizer : public rcppsw::er::client<los_visualizer> {
 public:
  explicit los_visualizer(argos::CQTOpenGLUserFunctions* qt)
      : ER_CLIENT_INIT("fordyca.support.los_visualizer"), m_qt(qt) {}

  los_visualizer(const los_visualizer& op) = delete;
  los_visualizer& operator=(const los_visualizer& op) = delete;

  /**
   * @brief Draw visualizations related to block LOS:
   *
   * @param block The LOS to visualize.
   * @param grid_resolution The grid resolution for the arena.
   */
  void draw(const representation::line_of_sight* const los,
            double grid_resolution);

  // clang-format off
  argos::CQTOpenGLUserFunctions* const m_qt{nullptr};
  // clang-format on
};

NS_END(fordyca, support);

#endif /* INCLUDE_FORDYCA_SUPPORT_LOS_VISUALIZER_HPP_ */
