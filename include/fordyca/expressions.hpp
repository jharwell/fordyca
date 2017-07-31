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
#include <vector>
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

  double calc(void) {
    return set_result(m_rho * last_result() + m_delta);
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
  block_utility(const argos::CVector2& block_loc,
                const argos::CVector2& nest_loc) :
      mc_block_loc(block_loc),
      mc_nest_loc(nest_loc) {}

  /* member functions */
  double calc(const argos::CVector2& rloc, double density) {
    return set_result(((mc_block_loc - mc_nest_loc).Length() /
                       (mc_block_loc - rloc).Length()) * std::exp(-density));
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
class existing_cache_utility: public rcppsw::math::expression<double> {
 public:
  existing_cache_utility(const argos::CVector2& cache_loc,
                const argos::CVector2& nest_loc) :
      mc_cache_loc(cache_loc), mc_nest_loc(nest_loc) {}

  double calc(const argos::CVector2& rloc, double density, size_t n_blocks) {
    return set_result((std::exp(-density) * n_blocks) /
                      ((mc_cache_loc - rloc).Length() *
                       (mc_cache_loc - mc_nest_loc).Length()));
  }

 private:
  const argos::CVector2 mc_cache_loc;
  const argos::CVector2 mc_nest_loc;
};

class new_cache_utility: public rcppsw::math::expression<double> {
 public:
  /* constructors */
  new_cache_utility(const argos::CVector2& cache_loc,
                    const argos::CVector2& nest_loc) :
      mc_cache_loc(cache_loc), mc_nest_loc(nest_loc) {}

  double calc(const argos::CVector2& rloc,
              const argos::CVector2& nearest_cache) {
    return set_result(nearest_cache.Length() /
                      ((mc_cache_loc - rloc).Length() * (rloc - (rloc - mc_nest_loc)/2)).Length());
  }
 private:
  const argos::CVector2 mc_cache_loc;
  const argos::CVector2 mc_nest_loc;
};
NS_END(harvest);

NS_START(tasks);
class partition_probability: public rcppsw::math::expression<double> {
 public:
  /* constructors */
  explicit partition_probability(double reactivity) :
      m_reactivity(reactivity) {}

  double calc(double task_time_estimate, double subtask1_time_estimate,
              double subtask2_time_estimate) {
    if (task_time_estimate > subtask1_time_estimate + subtask2_time_estimate) {
      double res = 1 + std::exp(-m_reactivity *
                                ((task_time_estimate /
                                  (subtask1_time_estimate + subtask2_time_estimate)) - 1));
      return set_result(1/res);
    } else {
      double res = 1 + std::exp(-m_reactivity * (1 -
                                                 ((subtask1_time_estimate + subtask2_time_estimate) /
                                                  task_time_estimate)));
      return set_result(1/res);
    }
  }
 private:
  double m_reactivity;
};

class time_estimate : public rcppsw::math::expression<double> {
 public:
  /* constructors */
  explicit time_estimate(double alpha) : m_alpha(alpha) {}

  /* member functions */
  double calc(double last_measure) {
    return set_result(1 - m_alpha * last_result() + m_alpha * last_measure);
  }

 private:
  double m_alpha;
};

class abort_probability: public rcppsw::math::expression<double> {
 public:
  /* constructors */
  abort_probability(double reactivity, double offset) :
      m_reactivity(reactivity), m_offset(offset) {}

  /* member functions */
  double calc(double exec_time,
              const time_estimate& whole_task,
              const time_estimate& subtask1,
              const time_estimate& subtask2) {
    double omega = m_reactivity * ((exec_time - whole_task.last_result())/
                                    (subtask1.last_result() + subtask2.last_result()) + m_offset);
    return set_result(1/(1 + std::exp(omega)));
  }

 private:
  double m_reactivity;
  double m_offset;
};

NS_END(tasks);

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_EXPRESSIONS_HPP_ */
