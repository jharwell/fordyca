/**
 * @file sub_area_utility.hpp
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

#ifndef INCLUDE_FORDYCA_MATH_SUB_AREA_UTILITY_HPP_
#define INCLUDE_FORDYCA_MATH_SUB_AREA_UTILITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>

#include <argos3/core/utility/math/vector2.h>
#include "fordyca/math/sub_area_poa.hpp"
#include "rcppsw/common/common.hpp"
#include "rcppsw/math/expression.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Class Definitions
 ********************************************p**********************************/
/**
 * @brief Calculates the utility that a particular sub-area will have to a
 * specific robot.
 *
 * Depends on:
 *
 * - Sub area probability of interest.
 * - Sub area distance to nest.
 * - Current robot position.
 */
class sub_area_utility : public rcppsw::math::expression<double> {
 public:
  sub_area_utility(const argos::CVector2& area_center,
                   const argos::CVector2& nest_center,
                   size_t squares)
      : expression(),
        m_squares(squares),
        m_unexplored(squares),
        mc_center(area_center),
        mc_nest(nest_center),
        m_poa(area_center, nest_center) {}

  double calc(const argos::CVector2& rloc,
              const std::vector<argos::CVector2>& caches) {
    double sum = 0;
    std::for_each(caches.begin(),
                  caches.end(),
                  [&](const argos::CVector2& cache) {
                    sum += (cache - mc_nest).Length();
                  });

    return set_result(m_poa.calc(sum) *
                      (m_unexplored / (rloc - mc_center).Length()));
  }
  void update_explored(size_t explored) { m_unexplored = m_squares - explored; }

 private:
  size_t m_squares;
  size_t m_unexplored;
  const argos::CVector2 mc_center;
  const argos::CVector2 mc_nest;
  sub_area_poa m_poa;
};

NS_END(math, fordyca);

#endif /* INCLUDE_FORDYCA_MATH_SUB_AREA_UTILITY_HPP_ */
