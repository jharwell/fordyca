/**
 * @file cache_site_selector.cpp
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
#include "fordyca/controller/depth2/cache_site_selector.hpp"
#include "fordyca/controller/cache_selection_matrix.hpp"
#include "fordyca/representation/perceived_cache.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/perceived_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_site_selector::cache_site_selector(
    const controller::cache_selection_matrix* const matrix)
    : client("fordyca.controller.depth2.cache_site_selector"),
      mc_matrix(matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
argos::CVector2 cache_site_selector::calc_best(
    const cache_list& known_caches,
    const block_list& known_blocks,
    argos::CVector2 robot_loc) {
  constraint_set constraints;

  /*
   * @bug If there are no constraints on the problem, the COBYLA method hangs,
   * so we need to use another method that the NLopt website is for
   * unconstrained non-linear optimization.
   *
   * This should probably be fixed, or at least understood WHY it is happening.
   */
  nlopt::opt alg1(nlopt::algorithm::LN_COBYLA, 2);
  nlopt::opt alg2(nlopt::algorithm::LN_PRAXIS, 2);
  if (known_caches.empty() && known_blocks.empty()) {
    m_alg = &alg2;
  } else {
    m_alg = &alg1;
    constraints_create(known_caches, known_blocks, &constraints);
    ER_INFO("Calculated %zu cache constraints, %zu block constraints",
            constraints.first.size(), constraints.second.size());
  }

  argos::CVector2 nest_loc = boost::get<argos::CVector2>(
      mc_matrix->find("nest_loc")->second);
  struct site_utility_data utility_data = {robot_loc, nest_loc};
  m_alg->set_max_objective(&__site_utility_func, &utility_data);
  m_alg->set_xtol_rel(kUTILITY_TOL);

  /* Initial guess: Halfway between robot and nest */
  argos::CVector2 tmp = (robot_loc + nest_loc) / 2.0;
  std::vector<double> best = {tmp.GetX(), tmp.GetY()};
  double max_utility;

  /* Do it! */
  nlopt::result res = m_alg->optimize(best, max_utility);
  ER_ASSERT(res >= 1, "NLopt failed with code %d", res);
  ER_DEBUG("NLopt return code: %d", res);
  ER_INFO("Selected cache site @(%f, %f), utility=%f", best[0], best[1],
          max_utility);
  return argos::CVector2(best[0], best[1]);
} /* calc_best() */

__rcsw_pure double __cache_constraint_func(const std::vector<double>& x,
                                           std::vector<double>& ,
                                           void *data) {
  cache_site_selector::cache_constraint_data* c =
      reinterpret_cast<cache_site_selector::cache_constraint_data*>(data);
  return c->cache_prox_dist - (argos::CVector2(x[0], x[1]) -
                               c->cache->real_loc()).Length();
} /* __cache_constraint_func() */

__rcsw_pure double __block_constraint_func(const std::vector<double>& x,
                                           std::vector<double>& ,
                                           void *data) {
  cache_site_selector::block_constraint_data* c =
      reinterpret_cast<cache_site_selector::block_constraint_data*>(data);
  return c->block_prox_dist - (argos::CVector2(x[0], x[1]) -
                               c->block->real_loc()).Length();
} /* __block_constraint_func() */

__rcsw_pure double __site_utility_func(const std::vector<double>& x,
                                       std::vector<double>& ,
                                       void *data) {
  cache_site_selector::site_utility_data* d =
      reinterpret_cast<cache_site_selector::site_utility_data*>(data);
  double robot_dist = (argos::CVector2(x[0], x[1]) - d->robot_loc).Length();
  argos::CVector2 nest_midpoint = (d->robot_loc + d->nest_loc) / 2;
  double nest_dist = (argos::CVector2(x[0], x[1]) - nest_midpoint).Length();
  return 1.0 / (robot_dist * nest_dist);
} /* __site_utility_func() */

void cache_site_selector::constraints_create(const cache_list& known_caches,
                                             const block_list& known_blocks,
                                             constraint_set* const constraints) {
  for (auto &c : known_caches) {
    constraints->first.push_back({c.get(),
            boost::get<double>(mc_matrix->find("cache_prox_dist")->second)});
  } /* for(&c..) */
  for (auto &b : known_blocks) {
    constraints->second.push_back({b.get(),
            boost::get<double>(mc_matrix->find("block_prox_dist")->second)});
  } /* for(&c..) */

  for (size_t i = 0; i < constraints->first.size(); ++i) {
    m_alg->add_inequality_constraint(__cache_constraint_func,
                                    &constraints->first[i],
                                    kCACHE_CONSTRAINT_TOL);
  } /* for(i..) */
  for (size_t i = 0; i < constraints->second.size(); ++i) {
    m_alg->add_inequality_constraint(__block_constraint_func,
                                    &constraints->second[i],
                                    kBLOCK_CONSTRAINT_TOL);
  } /* for(i..) */
} /* constraints_create() */

NS_END(depth2, controller, fordyca);
