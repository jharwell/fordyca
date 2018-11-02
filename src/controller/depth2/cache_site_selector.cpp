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

#include "fordyca/controller/cache_selection_matrix.hpp"
#include "fordyca/representation/perceived_cache.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/perceived_block.hpp"
#include "fordyca/math/cache_site_utility.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);
namespace rmath = rcppsw::math;

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
  std::vector<double> point;
  opt_initialize(known_caches, known_blocks, robot_loc, &constraints,
                 &point);
  double max_utility;

  /*
   * @bug Sometimes NLopt just fails with a generic error code and I don't
   * know why. This should be investigated and fixed, but for now this works.
   */
  try {
    nlopt::result res = m_alg.optimize(point, max_utility);
    ER_ASSERT(res >= 1, "NLopt failed with code %d", res);
    ER_INFO("NLopt return code: %d", res);
  } catch (std::runtime_error) {
    ER_WARN("NLopt failed");
  }

  ER_INFO("Selected cache site @(%f, %f), utility=%f", point[0], point[1],
          max_utility);
  uint cc_count = std::accumulate(std::begin(m_cc_violations),
                             std::end(m_cc_violations),
                             0);
  uint bc_count = std::accumulate(std::begin(m_bc_violations),
                                  std::end(m_bc_violations),
                                  0);

  ER_INFO("%u cache, %u block, %u nest constraint violations on cache site",
          cc_count, bc_count, m_nc_violations);

  return argos::CVector2(point[0], point[1]);
} /* calc_best() */

void cache_site_selector::constraints_create(const cache_list& known_caches,
                                             const block_list& known_blocks,
                                             const argos::CVector2& nest_loc,
                                             constraint_set* const constraints) {
  for (auto &c : known_caches) {
    std::get<0>(*constraints).push_back({c.get(),
            this,
            boost::get<double>(mc_matrix->find("cache_prox_dist")->second)});
  } /* for(&c..) */

  for (auto &b : known_blocks) {
    std::get<1>(*constraints).push_back({b.get(),
            this,
            boost::get<double>(mc_matrix->find("block_prox_dist")->second)});
  } /* for(&c..) */

  std::get<2>(*constraints).push_back({nest_loc,
          this,
          boost::get<double>(mc_matrix->find("nest_prox_dist")->second)});

  for (size_t i = 0; i < std::get<0>(*constraints).size(); ++i) {
    m_alg.add_inequality_constraint(__cache_constraint_func,
                                     &std::get<0>(*constraints)[i],
                                    kCACHE_CONSTRAINT_TOL);
  } /* for(i..) */
  for (size_t i = 0; i < std::get<1>(*constraints).size(); ++i) {
    m_alg.add_inequality_constraint(__block_constraint_func,
                                     &std::get<1>(*constraints)[i],
                                    kBLOCK_CONSTRAINT_TOL);
  } /* for(i..) */

  m_alg.add_inequality_constraint(__nest_constraint_func,
                                   &std::get<2>(*constraints)[0],
                                   kNEST_CONSTRAINT_TOL);
} /* constraints_create() */

void cache_site_selector::opt_initialize(
    const cache_list& known_caches,
    const block_list& known_blocks,
    argos::CVector2 robot_loc,
    constraint_set* const constraints,
    std::vector<double>* const initial_guess) {
    argos::CVector2 nest_loc = boost::get<argos::CVector2>(
      mc_matrix->find("nest_loc")->second);

  std::string baccum;
  std::for_each(known_blocks.begin(), known_blocks.end(), [&](const auto& b) {
      baccum += "b" + std::to_string(b->id()) + "->(" +
                std::to_string(b->discrete_loc().first) + "," +
                std::to_string(b->discrete_loc().second) + "),";
    });

  std::string caccum;
  std::for_each(known_caches.begin(), known_caches.end(), [&](const auto& c) {
      caccum += "c" + std::to_string(c->id()) + "->(" +
                std::to_string(c->discrete_loc().first) + "," +
                std::to_string(c->discrete_loc().second) + "),";
    });

  ER_INFO("Known blocks: [%s]", baccum.c_str());
  ER_INFO("Known caches: [%s]", caccum.c_str());

  /*
   * If there are no constraints on the problem, the COBYLA method hangs, BUT
   * that is OK because we always have at least the nest proximity constraint.
   */
  constraints_create(known_caches, known_blocks, nest_loc, constraints);
  ER_INFO("Calculated %zu cache, %zu block, %zu nest constraints",
          std::get<0>(*constraints).size(),
          std::get<1>(*constraints).size(),
          std::get<2>(*constraints).size());
  m_cc_violations.reserve(std::get<0>(*constraints).size());
  m_bc_violations.reserve(std::get<1>(*constraints).size());

  auto xrange = boost::get<rmath::range<uint>>(mc_matrix->find("site_xrange")->second);
  auto yrange = boost::get<rmath::range<uint>>(mc_matrix->find("site_yrange")->second);
  struct site_utility_data utility_data = {robot_loc, nest_loc};
  m_alg.set_max_objective(&__site_utility_func, &utility_data);
  m_alg.set_ftol_rel(kUTILITY_TOL);
  m_alg.set_stopval(1000000);
  m_alg.set_lower_bounds({static_cast<double>(xrange.get_min()),
          static_cast<double>(yrange.get_min())});
  m_alg.set_upper_bounds({static_cast<double>(xrange.get_max()),
          static_cast<double>(yrange.get_max())});
  m_alg.set_maxeval(kMAX_ITERATIONS);
  m_alg.set_default_initial_step({1.0, 1.0});

  /* Initial guess: random point in the arena */
  uint x = std::min((std::rand() % xrange.get_min()) + 1, xrange.get_max());
  uint y = std::min((std::rand() % yrange.get_min()) + 1, yrange.get_max());
  *initial_guess = {static_cast<double>(x), static_cast<double>(y)};
  ER_INFO("Initial guess: (%u,%u), xrange=[%u-%u], yrange=[%u-%u]",
          x,
          y,
          xrange.get_min(),
          xrange.get_max(),
          yrange.get_min(),
          yrange.get_max());
} /* opt_initialize() */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
__rcsw_pure double __cache_constraint_func(const std::vector<double>& x,
                                           std::vector<double>& ,
                                           void *data) {
  if (std::isnan(x[0]) || std::isnan(x[1])) {
    return std::numeric_limits<double>::max();
  }
  cache_site_selector::cache_constraint_data* c =
      reinterpret_cast<cache_site_selector::cache_constraint_data*>(data);
  double val = c->cache_prox_dist - (argos::CVector2(x[0], x[1]) -
                                     c->cache->real_loc()).Length();

  if (val > 0) {
    c->selector->cc_violated(c->cache->id());
  } else {
    c->selector->cc_satisfied(c->cache->id());
  }
  return val;
} /* __cache_constraint_func() */

__rcsw_pure double __nest_constraint_func(const std::vector<double>& x,
                                           std::vector<double>& ,
                                           void *data) {
  if (std::isnan(x[0]) || std::isnan(x[1])) {
    return std::numeric_limits<double>::max();
  }
  cache_site_selector::nest_constraint_data* c =
      reinterpret_cast<cache_site_selector::nest_constraint_data*>(data);
  double val = c->nest_prox_dist - (argos::CVector2(x[0], x[1]) -
                                    c->nest_loc).Length();
  if (val > 0) {
    c->selector->nc_violated();
  } else {
    c->selector->nc_satisfied();
  }

  return val;
} /* __nest_constraint_func() */

__rcsw_pure double __block_constraint_func(const std::vector<double>& x,
                                                std::vector<double>& ,
                                                void *data) {
  if (std::isnan(x[0]) || std::isnan(x[1])) {
    return std::numeric_limits<double>::max();
  }
  cache_site_selector::block_constraint_data* c =
      reinterpret_cast<cache_site_selector::block_constraint_data*>(data);
  double val = c->block_prox_dist - (argos::CVector2(x[0], x[1]) -
                        c->block->real_loc()).Length();
  if (val > 0) {
    c->selector->bc_violated(c->block->id());
  } else {
    c->selector->bc_satisfied(c->block->id());
  }
  return val;
} /* __block_constraint_func() */

__rcsw_pure double __site_utility_func(const std::vector<double>& x,
                                       std::vector<double>& ,
                                       void *data) {
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
  argos::CVector2 point(x[0], x[1]);

  return math::cache_site_utility(d->robot_loc, d->nest_loc)(point);
} /* __site_utility_func() */

NS_END(depth2, controller, fordyca);
