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
      mc_matrix(matrix),
      m_alg(nlopt::algorithm::LN_COBYLA, 2) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
argos::CVector2 cache_site_selector::calc_best(
    const cache_list& known_caches,
    const block_list& known_blocks,
    argos::CVector2 robot_loc) {
  /* Set problem constraints add object function */

  volatile auto constraints = constraints_create(known_caches, known_blocks);
  argos::CVector2 nest_loc = boost::get<argos::CVector2>(
      mc_matrix->find("nest_loc")->second);
  struct site_utility_data utility_data = {robot_loc, nest_loc};
  m_alg.set_max_objective(&__site_utility_func, &utility_data);
  m_alg.set_xtol_rel(kUTILITY_TOL);

  /* Initial guess: Halfway between robot and nest */
  argos::CVector2 tmp = (robot_loc + nest_loc) / 2.0;
  std::vector<double> best = {tmp.GetX(), tmp.GetY()};
  double max_utility;

  /* Do it! */
  nlopt::result res = m_alg.optimize(best, max_utility);
  ER_ASSERT(0 == res, "NLopt failed with code %d", res);
  ER_INFO("Best utility: cache_site at (%f, %f) [%f]", best[0], best[1],
          max_utility);
  return argos::CVector2(best[0], best[1]);
} /* calc_best() */

double __cache_constraint_func(const std::vector<double>& x,
                                            std::vector<double>& ,
                                            void *data) {
  cache_site_selector::cache_constraint_data* c =
      reinterpret_cast<cache_site_selector::cache_constraint_data*>(data);
    return (argos::CVector2(x[0], x[1]) -
            c->cache->real_loc()).Length() - c->cache_prox_dist;
} /* __cache_constraint_func() */

double __block_constraint_func(const std::vector<double>& x,
                               std::vector<double>& ,
                               void *data) {
  cache_site_selector::block_constraint_data* c =
      reinterpret_cast<cache_site_selector::block_constraint_data*>(data);
  return (argos::CVector2(x[0], x[1]) -
          c->block->real_loc()).Length() - c->block_prox_dist;
} /* __block_constraint_func() */

double __site_utility_func(const std::vector<double>& x,
                               std::vector<double>& ,
                               void *data) {
  cache_site_selector::site_utility_data* d =
      reinterpret_cast<cache_site_selector::site_utility_data*>(data);
  double robot_dist = (argos::CVector2(x[0], x[1]) - d->robot_loc).Length();
  double nest_dist = (argos::CVector2(x[0], x[1]) - d->nest_loc).Length();
  return 1.0 / (robot_dist * nest_dist);
} /* __site_utility_func() */

cache_site_selector::constraint_return_type cache_site_selector::constraints_create(
    const cache_list& known_caches,
    const block_list& known_blocks) {
  std::vector<block_constraint_data> block_constraints;
  std::vector<cache_constraint_data> cache_constraints;

  for (auto &c : known_caches) {
    cache_constraints.push_back({c.get(),
            boost::get<double>(mc_matrix->find("cache_prox_dist")->second)});
  } /* for(&c..) */
  for (auto &b : known_blocks) {
    block_constraints.push_back({b.get(),
            boost::get<double>(mc_matrix->find("block_prox_dist")->second)});
  } /* for(&c..) */

  for (size_t i = 0; i < cache_constraints.size(); ++i) {
    m_alg.add_inequality_constraint(__cache_constraint_func,
                                    &cache_constraints[i],
                                    kCACHE_CONSTRAINT_TOL);
  } /* for(i..) */
  for (size_t i = 0; i < block_constraints.size(); ++i) {
    m_alg.add_inequality_constraint(__block_constraint_func,
                                    &block_constraints[i],
                                    kBLOCK_CONSTRAINT_TOL);
  } /* for(i..) */
  return constraint_return_type(cache_constraints, block_constraints);
} /* constraints_create() */

NS_END(depth2, controller, fordyca);
