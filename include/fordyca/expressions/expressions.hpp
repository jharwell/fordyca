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

#ifndef INCLUDE_FORDYCA_EXPRESSIONS_EXPRESSIONS_HPP_
#define INCLUDE_FORDYCA_EXPRESSIONS_EXPRESSIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include <cmath>
#include <vector>
#include <algorithm>
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/math/expression.hpp"
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, expressions);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Calculates the probability that a particular sub-area will be of
 * interest to a robot.
 *
 * Depends on:
 *
 * - Sub area distance to nest. This emphasizes sub areas that are further away,
 *   encouraging bringing items from further away before bring items that are
 *   closer, facilitating a general movement of items closer to the nest.
 *
 * - Total distance from all known caches in the subarea to nest. This
 *   emphasizes exploiting existing caches when they exist, rather than go
 *   exploring.
 *
 * - TODO: take pheromones into account?
 */
class sub_area_poa: public rcppsw::math::expression<double> {
 public:
  sub_area_poa(const argos::CVector2& area_center,
               const argos::CVector2& nest_center) :
      mc_center(area_center), mc_nest(nest_center) {}

  double calc(double caches_dist) {
    return set_result((mc_center - mc_nest).Length() / caches_dist);
  }

 private:
  const argos::CVector2 mc_center;
  const argos::CVector2 mc_nest;
};

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
class sub_area_utility: public rcppsw::math::expression<double> {
 public:
  sub_area_utility(const argos::CVector2& area_center,
                   const argos::CVector2& nest_center,
                   size_t squares) :
      expression(),
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

    return set_result(m_poa.calc(sum) * (m_unexplored /
                                         (rloc - mc_center).Length()));
  }
  void update_explored(size_t explored) { m_unexplored = m_squares - explored; }

 private:
  size_t m_squares;
  size_t m_unexplored;
  const argos::CVector2 mc_center;
  const argos::CVector2 mc_nest;
  sub_area_poa m_poa;
};

/**
 * @brief Calculates the pheromone density associated with a block or cache,
 * which decays over time.
 *
 * Depends on:
 *
 * - The pheromone decay parameter.
 * - The previous value of the pheromone density.
 */
class pheromone_density: public rcppsw::math::expression<double> {
 public:
  pheromone_density(void) : pheromone_density(0.0) {}
  explicit pheromone_density(double rho) :
      expression(), m_delta(0), m_rho(rho) {}

  void rho(double rho) { m_rho = rho; }

  double calc(void) {
    double res = set_result((1.0 - m_rho) * last_result() + m_delta);
    m_delta = 0;
    return res;
  }
  void add_pheromone(double val) {
    m_delta += val;
  }

 private:
  double m_delta;
  double m_rho;
};

NS_START(forage);

/**
 * @brief Calculates the utility associated with a known block/cache, as part of
 * a robot's decision on whether or not to go and attempt to pick it up (block) or
 * pickup a block from it (cache).
 *
 * Depends on:
 *
 * - Distance of block to nest (Further is better).
 * - Distance of block to robot's current position (closer is better).
 * - Pheromone density associated with the block information (higher is better).
 */
class block_utility: public rcppsw::math::expression<double>  {
 public:
  block_utility(const argos::CVector2& block_loc,
                const argos::CVector2& nest_loc) :
      mc_block_loc(block_loc),
      mc_nest_loc(nest_loc) {}

  double calc(const argos::CVector2& rloc, double density) {
    return set_result(((mc_block_loc - mc_nest_loc).Length() /
                       (mc_block_loc - rloc).Length()) * std::exp(-density));
  }

 private:
  const argos::CVector2 mc_block_loc;
  const argos::CVector2 mc_nest_loc;
};


typedef block_utility cache_utility;

NS_END(forage);
NS_START(harvest);

/**
 * @brief Calculates the utility associated with an existing cache that the
 * robot knows about.
 *
 * Depends on:
 *
 * - Distance of cache to nest (closer is better).
 * - Distance of cache to robot's current position (closer is better).
 * - # of blocks believed to be in the cache (more is better).
 * - Pheromone density associated with the cache information (higher is
 *   better).
 */
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

/**
 * @brief Calculates the utility associated with a new cache to a robot as part
 * of its decision process for what to do with a block once it has picked it up.
 *
 * Depends on:
 *
 * - Distance of perspective site to the nest (closer is better).
 * - Distance of perspective site to robot's current location (closer is
 * - better).
 * - Distance to nearest know cache (further is better).
 */
class new_cache_utility: public rcppsw::math::expression<double> {
 public:
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

/**
 * @brief Calculates the probability that a robot partitions its current task.
 *
 * Depends on:
 *
 * - The robot's time estimates of how long it takes to complete each of the two
 *   subtasks, as well as an estimate of how long it takes to complete the
 *   unpartitioned task.
 *
 * - The reactivity parameter: how sensitive should robots be to abrupt changes
 *   in the estimates?
 */
class partition_probability: public rcppsw::math::expression<double> {
 public:
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

/**
 * @brief Calculates an estimate of how long a task will take.
 *
 * Depends on:
 *
 * - Alpha: How much weight to give the past estimate, and how much to give the
 *   new measurement?
 *
 * - The last value of the time estimate.
 */
class time_estimate : public rcppsw::math::expression<double> {
 public:
  explicit time_estimate(double alpha) : m_alpha(alpha) {}

  double calc(double last_measure) {
    return set_result(1 - m_alpha * last_result() + m_alpha * last_measure);
  }

 private:
  double m_alpha;
};

/**
 * @brief Calculates the probability that a robot will abort the task it is
 * currently working on.
 *
 * Depends on:
 *
 * - The reactivity parameter: How sensitive should robots be to abrupt changes
 *   in task estimates/execution times.
 * - Time estimates of the unpartitioned and two partitioned tasks.
 * - How long the robot has spent executing the current task.
 * - The offset parameter: Another parameter whose purpose I'm not quite sure
 *   of.
 */
class abort_probability: public rcppsw::math::expression<double> {
 public:
  abort_probability(double reactivity, double offset) :
      m_reactivity(reactivity), m_offset(offset) {}

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
NS_END(expressions, fordyca);

#endif /* INCLUDE_FORDYCA_EXPRESSIONS_EXPRESSIONS_HPP_ */
