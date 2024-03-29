/**
 * \file cache_site_selector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/fsm/d2/cache_site_selector.hpp"

#include "cosm/arena/repr/base_cache.hpp"

#include "fordyca/controller/cognitive/cache_sel_matrix.hpp"
#include "fordyca/math/cache_site_utility.hpp"
#include "fordyca/subsystem/perception/ds/dp_block_map.hpp"
#include "fordyca/subsystem/perception/ds/dp_cache_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d2);
using cselm = controller::cognitive::cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_site_selector::cache_site_selector(
    const controller::cognitive::cache_sel_matrix* const matrix)
    : ER_CLIENT_INIT("fordyca.controller.d2.cache_site_selector"),
      mc_matrix(matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<rmath::vector2d>
cache_site_selector::operator()(const cads::bcache_vectorno& known_caches,
                                rmath::vector2d position,
                                rmath::rng* rng) {
  double max_utility;
  std::vector<double> point;
  struct site_utility_data u;
  rmath::vector2d site;
  opt_init_conditions init_cond{ known_caches, position };
  opt_initialize(&init_cond, &u, &point, rng);

  /*
   * @bug Sometimes NLopt just fails with a generic error code and I don't
   * know why. I *think* it is because I hand it an infeasible point to start
   * with (i.e. one that violates the existing constraints). Should probably fix
   * so exception catching is not necessary, but for now this seems to work.
   */
  try {
    nlopt::result res = m_alg.optimize(point, max_utility);
    ER_INFO("NLopt returned: '%s', max_utility=%f",
            nlopt_ret_str(res).c_str(),
            max_utility);
    ER_ASSERT(res >= 1, "NLopt failed with code %d", res);
    m_nlopt_res = res;
  } catch (std::runtime_error&) {
    ER_FATAL_SENTINEL("NLopt failed");
    return boost::optional<rmath::vector2d>();
  }
  site.set(point[0], point[1]);

  ER_INFO("Computed cache site@%s", rcppsw::to_string(site).c_str());

  bool site_ok = verify_site(site, known_caches);
  bool strict =
      std::get<bool>(mc_matrix->find(cselm::kStrictConstraints)->second);

  if (site_ok || (!site_ok && !strict)) {
    return boost::make_optional(site);
  } else {
    ER_WARN("Discard cache site@%s: violates constraints",
            rcppsw::to_string(site).c_str());
    return boost::optional<rmath::vector2d>();
  }
} /* operator()() */

bool cache_site_selector::verify_site(
    const rmath::vector2d& site,
    const cads::bcache_vectorno& known_caches) const {
  const nest_constraint_data* ndata = &std::get<1>(m_constraints)[0];

  /* check distances to known caches */
  for (const auto& c : known_caches) {
    ER_CHECK(rtypes::spatial_dist((c->rcenter2D() - site).length()) >=
                 std::get<0>(m_constraints)[0].cache_prox,
             "Cache site@%s too close to cache%d (%f <= %f)",
             rcppsw::to_string(site).c_str(),
             c->id().v(),
             (c->rcenter2D() - site).length(),
             std::get<0>(m_constraints)[0].cache_prox.v());
  } /* for(&c..) */

  /* check distance to nest center */
  ER_CHECK(rtypes::spatial_dist((ndata->nest_loc - site).length()) >=
               ndata->nest_prox,
           "Cache site@%s too close to nest (%f <= %f)",
           rcppsw::to_string(site).c_str(),
           (ndata->nest_loc - site).length(),
           ndata->nest_prox.v());

  return true;

error:
  return false;
} /* verify_site() */

void cache_site_selector::opt_initialize(const opt_init_conditions* cond,
                                         struct site_utility_data* utility_data,
                                         std::vector<double>* initial_guess,
                                         rmath::rng* rng) {
  rmath::vector2d nest_loc =
      std::get<rmath::vector2d>(mc_matrix->find(cselm::kNestLoc)->second);

  ER_INFO("Known caches: [%s]", rcppsw::to_string(cond->known_caches).c_str());

  /*
   * If there are no constraints on the problem, the COBYLA method hangs, BUT
   * that is OK because we always have at least the nest proximity constraint.
   */
  constraints_create(cond->known_caches, nest_loc);
  ER_INFO("Calculated %zu cache, %zu nest constraints",
          std::get<0>(m_constraints).size(),
          std::get<1>(m_constraints).size());

  auto xrange =
      std::get<rmath::rangez>(mc_matrix->find(cselm::kSiteXRange)->second);
  auto yrange =
      std::get<rmath::rangez>(mc_matrix->find(cselm::kSiteYRange)->second);
  *utility_data = { cond->position, nest_loc };
  m_alg.set_max_objective(&__site_utility_func, utility_data);
  m_alg.set_ftol_rel(kUTILITY_TOL);
  m_alg.set_stopval(1000000);
  m_alg.set_lower_bounds(
      { static_cast<double>(xrange.lb()), static_cast<double>(yrange.lb()) });
  m_alg.set_upper_bounds(
      { static_cast<double>(xrange.ub()), static_cast<double>(yrange.ub()) });
  m_alg.set_maxeval(kMAX_ITERATIONS);
  m_alg.set_default_initial_step({ 1.0, 1.0 });

  /* Initial guess: random point in the arena */
  std::uniform_int_distribution<> xdist(xrange.lb(), xrange.ub());
  std::uniform_int_distribution<> ydist(yrange.lb(), yrange.ub());
  uint x = rng->uniform(xrange);
  uint y = rng->uniform(yrange);

  *initial_guess = { static_cast<double>(x), static_cast<double>(y) };
  ER_INFO("Initial guess: (%u,%u), xrange=%s, yrange=%s",
          x,
          y,
          xrange.to_str().c_str(),
          yrange.to_str().c_str());
} /* opt_initialize() */

void cache_site_selector::constraints_create(
    const cads::bcache_vectorno& known_caches,
    const rmath::vector2d& nest_loc) {
  for (const auto& c : known_caches) {
    std::get<0>(m_constraints)
        .push_back({ c,
                     this,
                     std::get<rtypes::spatial_dist>(
                         mc_matrix->find(cselm::kCacheProxDist)->second) });
  } /* for(&c..) */

  std::get<1>(m_constraints)
      .push_back({ nest_loc,
                   this,
                   std::get<rtypes::spatial_dist>(
                       mc_matrix->find(cselm::kNestProxDist)->second) });

  for (auto& c : std::get<0>(m_constraints)) {
    m_alg.add_inequality_constraint(
        __cache_constraint_func, &c, kCACHE_CONSTRAINT_TOL);
  } /* for(c..) */

  m_alg.add_inequality_constraint(__nest_constraint_func,
                                  &std::get<1>(m_constraints)[0],
                                  kNEST_CONSTRAINT_TOL);
} /* constraints_create() */

std::string cache_site_selector::nlopt_ret_str(nlopt::result res) const {
  switch (res) {
    case nlopt::result::FAILURE:
      return "FAILURE";
    case nlopt::result::INVALID_ARGS:
      return "INVALID_ARGS";
    case nlopt::result::OUT_OF_MEMORY:
      return "OUT_OF_MEMORY";
    case nlopt::result::ROUNDOFF_LIMITED:
      return "ROUNDOFF_LIMITED";
    case nlopt::result::FORCED_STOP:
      return "FORCED_STOP";
    case nlopt::result::SUCCESS:
      return "SUCCESS";
    case nlopt::result::STOPVAL_REACHED:
      return "STOPVAL_REACHED";
    case nlopt::result::FTOL_REACHED:
      return "FTOL_REACHED";
    case nlopt::result::XTOL_REACHED:
      return "XTOL_REACHED";
    case nlopt::result::MAXEVAL_REACHED:
      return "MAXEVAL_REACHED";
    case nlopt::result::MAXTIME_REACHED:
      return "MAXTIME_REACHED";
      break;
    default:
      return "";
  } /* switch() */
} /* nlopt_ret_str() */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
double __cache_constraint_func(const std::vector<double>& x,
                               std::vector<double>&,
                               void* data) {
  if (std::isnan(x[0]) || std::isnan(x[1])) {
    return std::numeric_limits<double>::max();
  }
  auto* c = reinterpret_cast<cache_site_selector::cache_constraint_data*>(data);
  double val = c->cache_prox.v() -
               (rmath::vector2d(x[0], x[1]) - c->mc_cache->rcenter2D()).length();
  return val;
} /* __cache_constraint_func() */

double __nest_constraint_func(const std::vector<double>& x,
                              std::vector<double>&,
                              void* data) {
  if (std::isnan(x[0]) || std::isnan(x[1])) {
    return std::numeric_limits<double>::max();
  }
  auto* c = reinterpret_cast<cache_site_selector::nest_constraint_data*>(data);
  return c->nest_prox.v() - (rmath::vector2d(x[0], x[1]) - c->nest_loc).length();
} /* __nest_constraint_func() */

double __site_utility_func(const std::vector<double>& x,
                           std::vector<double>&,
                           void* data) {
  /*
   * \todo If for some reason we get a NaN point, return the worst possible
   * utility. Again this should probably not be necessary, but I don't know
   * enough about optimization theory to say for sure.
   */
  if (std::isnan(x[0]) || std::isnan(x[1])) {
    return std::numeric_limits<double>::min();
  }
  auto* d = reinterpret_cast<cache_site_selector::site_utility_data*>(data);
  rmath::vector2d point(x[0], x[1]);
  return math::cache_site_utility(d->position, d->nest_loc)(point);
} /* __site_utility_func() */

NS_END(d2, fsm, fordyca);
