/**
 * @file sub_area.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_SUB_AREA_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_SUB_AREA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include <vector>
#include "fordyca/expressions/expressions.hpp"
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, resprentation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Representation of a sub area of the arena to a specific
 * robot. Includes how much has been explored, how valuable this sub area is
 * considered to be to the robot, and known caches/blocks along with
 * freshness/staleness of information via pheromones.
 */
class sub_area {
 public:
  sub_area(size_t dim,
           const argos::CVector2& area_center,
           const argos::CVector2& nest_center)
      : m_dim(dim),
        m_center(area_center),
        m_utility(m_center, nest_center, m_dim * m_dim) {}

 private:
  size_t m_dim;
  argos::CVector2 m_center;
  expressions::sub_area_utility m_utility;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTION_SUB_AREA_HPP_ */
