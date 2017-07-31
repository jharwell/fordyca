/**
 * @file expressions.hpp
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

#ifndef INCLUDE_FORDYCA_EXPRESSIONS_HPP_
#define INCLUDE_FORDYCA_EXPRESSIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include <cmath>
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/math/expression.hpp"
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class pheremone_density: public rcppsw::math::expression<double> {
 public:
  explicit pheremone_density(double rho) :
      expression(), m_delta(0), m_rho(rho) {}

  double calc(void) const {
    return m_rho * last_res() + m_delta;
  }
  void add_pheremone(void) {
    ++m_delta;
  }

 private:
  double m_delta;
  double m_rho;
};

NS_START(forage);
class block_utility: public rcppsw::math::expression<double>  {
 public:
  /* constructors */
  block_utility(argos::CVector2 block_loc, argos::CVector2 nest_loc) :
      mc_block_loc(block_loc),
      mc_nest_loc(nest_loc) {}

  /* member functions */
  double calc(const argos::CVector2& rloc, double density) {
    return ((mc_block_loc - mc_nest_loc).Length() /
            (mc_block_loc - rloc).Length()) * std::exp(-density);
  }


 private:
  /* member functions */

  /* data members */
  const argos::CVector2 mc_block_loc;
  const argos::CVector2 mc_nest_loc;
};
typedef block_utility cache_utility;
NS_END(forage);

NS_START(harvest);

NS_END(harvest);

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_EXPRESSIONS_HPP_ */
