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
#include <random>

#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/dbg/dbg.hpp"
#include "fordyca/math/cache_site_utility.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/perceived_block.hpp"
#include "fordyca/representation/perceived_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);
using cselm = cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_site_selector::cache_site_selector(
    const controller::cache_sel_matrix* const matrix)
    : ER_CLIENT_INIT("fordyca.controller.depth2.cache_site_selector"),
      mc_matrix(matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d cache_site_selector::calc_best(
    const ds::cache_list& known_caches,
    const ds::block_list& known_blocks,
    rmath::vector2d position) {
  double max_utility;
  std::vector<double> point;
  struct site_utility_data u;
  rmath::vector2d site(-1, -1);
  opt_initialize(known_caches, known_blocks, position, &u, &point);

  /*
   * @bug Sometimes NLopt just fails with a generic error code and I don't
   * know why. This should be investigated and fixed, but for now this seems to
   * work.
   */
  try {
    nlopt::result res = m_alg.optimize(point, max_utility);
    ER_ASSERT(res >= 1, "NLopt failed with code %d", res);
    ER_INFO("NLopt return code: %d", res);
  } catch (std::runtime_error) {
    ER_WARN("NLopt failed");
    return site;
  }
  site.set(point[0], point[1]);
  ER_ASSERT(verify_site(site, known_caches, known_blocks),
            "Selected cache violates constraints");

  ER_INFO("Selected cache site @(%f, %f), utility=%f",
          point[0],
          point[1],
          max_utility);

  return site;
} /* calc_best() */

bool cache_site_selector::verify_site(const rmath::vector2d& site,
                                      const ds::cache_list& known_caches,
                                      const ds::block_list& known_blocks) const {
  for (auto& c : known_caches) {
    ER_ASSERT((c->real_loc() - site).length() >=
                  std::get<0>(m_constraints)[0].cache_prox_dist,
              "Cache site@%s too close to cache%d (%f <= %f)",
              site.to_str().c_str(),
              c->id(),
              (c->real_loc() - site).length(),
              std::get<0>(m_constraints)[0].cache_prox_dist);
  } /* for(&c..) */

  for (auto& b : known_blocks) {
    ER_ASSERT((b->real_loc() - site).length() >=
                  std::get<1>(m_constraints)[0].block_prox_dist,
              "Cache site@%s too close to block%d (%f <= %f)",
              site.to_str().c_str(),
              b->id(),
              (b->real_loc() - site).length(),
              std::get<1>(m_constraints)[0].block_prox_dist);
  } /* for(&b..) */
  const nest_constraint_data* ndata = &std::get<2>(m_constraints)[0];
  ER_ASSERT((ndata->nest_loc - site).length() >= ndata->nest_prox_dist,
            "Cache site@%s too close to nest (%f <= %f)",
            site.to_str().c_str(),
            (ndata->nest_loc - site).length(),
            ndata->nest_prox_dist);
  return true;
} /* verify_site() */

void cache_site_selector::opt_initialize(
    const ds::cache_list& known_caches,
    const ds::block_list& known_blocks,
    rmath::vector2d position,
    struct site_utility_data* const utility_data,
    std::vector<double>* const initial_guess) {
  rmath::vector2d nest_loc =
      boost::get<rmath::vector2d>(mc_matrix->find(cselm::kNestLoc)->second);

  ER_INFO("Known blocks: [%s]", dbg::blocks_list(known_blocks).c_str());
  ER_INFO("Known caches: [%s]", dbg::caches_list(known_caches).c_str());

  /*
   * If there are no constraints on the problem, the COBYLA method hangs, BUT
   * that is OK because we always have at least the nest proximity constraint.
   */
  constraints_create(known_caches, known_blocks, nest_loc);
  ER_INFO("Calculated %zu cache, %zu block, %zu nest constraints",
          std::get<0>(m_constraints).size(),
          std::get<1>(m_constraints).size(),
          std::get<2>(m_constraints).size());

  auto xrange =
      boost::get<rmath::rangeu>(mc_matrix->find(cselm::kSiteXRange)->second);
  auto yrange =
      boost::get<rmath::rangeu>(mc_matrix->find(cselm::kSiteYRange)->second);
  *utility_data = {position, nest_loc};
  m_alg.set_max_objective(&__site_utility_func, utility_data);
  m_alg.set_ftol_rel(kUTILITY_TOL);
  m_alg.set_stopval(1000000);
  m_alg.set_lower_bounds(
      {static_cast<double>(xrange.lb()), static_cast<double>(yrange.lb())});
  m_alg.set_upper_bounds(
      {static_cast<double>(xrange.ub()), static_cast<double>(yrange.ub())});
  m_alg.set_maxeval(kMAX_ITERATIONS);
  m_alg.set_default_initial_step({1.0, 1.0});

  /* Initial guess: random point in the arena */
  uint x = std::max(std::min((std::rand() % xrange.ub()) + 1, xrange.ub()),
                    xrange.lb());
  uint y = std::max(std::min((std::rand() % yrange.ub()) + 1, yrange.ub()),
                    yrange.lb());
  *initial_guess = {static_cast<double>(x), static_cast<double>(y)};
  ER_INFO("Initial guess: (%u,%u), xrange=%s, yrange=%s",
          x,
          y,
          xrange.to_str().c_str(),
          yrange.to_str().c_str());
} /* opt_initialize() */

void cache_site_selector::constraints_create(const ds::cache_list& known_caches,
                                             const ds::block_list& known_blocks,
                                             const rmath::vector2d& nest_loc) {
  for (auto& c : known_caches) {
    std::get<0>(m_constraints)
        .push_back({c.get(),
                    this,
                    boost::get<double>(
                        mc_matrix->find(cselm::kCacheProxDist)->second)});
  } /* for(&c..) */

  for (auto& b : known_blocks) {
    std::get<1>(m_constraints)
        .push_back({b.get(),
                    this,
                    boost::get<double>(
                        mc_matrix->find(cselm::kBlockProxDist)->second)});
  } /* for(&c..) */

  std::get<2>(m_constraints)
      .push_back(
          {nest_loc,
           this,
           boost::get<double>(mc_matrix->find(cselm::kNestProxDist)->second)});

  for (size_t i = 0; i < std::get<0>(m_constraints).size(); ++i) {
    m_alg.add_inequality_constraint(__cache_constraint_func,
                                    &std::get<0>(m_constraints)[i],
                                    kCACHE_CONSTRAINT_TOL);
  } /* for(i..) */
  for (size_t i = 0; i < std::get<1>(m_constraints).size(); ++i) {
    m_alg.add_inequality_constraint(__block_constraint_func,
                                    &std::get<1>(m_constraints)[i],
                                    kBLOCK_CONSTRAINT_TOL);
  } /* for(i..) */

  m_alg.add_inequality_constraint(__nest_constraint_func,
                                  &std::get<2>(m_constraints)[0],
                                  kNEST_CONSTRAINT_TOL);
} /* constraints_create() */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
__rcsw_pure double __cache_constraint_func(const std::vector<double>& x,
                                           std::vector<double>&,
                                           void* data) {
  if (std::isnan(x[0]) || std::isnan(x[1])) {
    return std::numeric_limits<double>::max();
  }
  cache_site_selector::cache_constraint_data* c =
      reinterpret_cast<cache_site_selector::cache_constraint_data*>(data);
  double val = c->cache_prox_dist -
               (rmath::vector2d(x[0], x[1]) - c->cache->real_loc()).length();
  return val;
} /* __cache_constraint_func() */

__rcsw_pure double __nest_constraint_func(const std::vector<double>& x,
                                          std::vector<double>&,
                                          void* data) {
  if (std::isnan(x[0]) || std::isnan(x[1])) {
    return std::numeric_limits<double>::max();
  }
  cache_site_selector::nest_constraint_data* c =
      reinterpret_cast<cache_site_selector::nest_constraint_data*>(data);
  double val =
      c->nest_prox_dist - (rmath::vector2d(x[0], x[1]) - c->nest_loc).length();
  return val;
} /* __nest_constraint_func() */

__rcsw_pure double __block_constraint_func(const std::vector<double>& x,
                                           std::vector<double>&,
                                           void* data) {
  if (std::isnan(x[0]) || std::isnan(x[1])) {
    return std::numeric_limits<double>::max();
  }
  cache_site_selector::block_constraint_data* c =
      reinterpret_cast<cache_site_selector::block_constraint_data*>(data);
  double val = c->block_prox_dist -
               (rmath::vector2d(x[0], x[1]) - c->block->real_loc()).length();
  return val;
} /* __block_constraint_func() */

__rcsw_pure double __site_utility_func(const std::vector<double>& x,
                                       std::vector<double>&,
                                       void* data) {
  /*
   * @todo If for some reason we get a NaN point, return the worst possible
   * utility. Again this should probably not be necessary, but I don't know
   * enough about optimization theory to say for sure.
   */
  if (std::isnan(x[0]) || std::isnan(x[1])) {
    return std::numeric_limits<double>::min();
  }
  cache_site_selector::site_utility_data* d =
      reinterpret_cast<cache_site_selector::site_utility_data*>(data);
  rmath::vector2d point(x[0], x[1]);
  return math::cache_site_utility(d->position, d->nest_loc)(point);
} /* __site_utility_func() */

NS_END(depth2, controller, fordyca);
