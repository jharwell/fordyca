/**
 * @file cache_site_selector.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_CACHE_SITE_SELECTOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_CACHE_SITE_SELECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <tuple>
#include <functional>
#include <vector>
#include <nlopt.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "fordyca/ds/dp_block_map.hpp"
#include "fordyca/ds/dp_cache_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
class cache_sel_matrix;
namespace rmath = rcppsw::math;
NS_START(depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_site_selector
 * @ingroup controller depth2
 *
 * @brief Selects the best cache site between the location of the block pickup
 * and the nest (ideally the halfway point), subject to constraints such as it
 * can't be too near other known blocks, known caches, or the nest.
 */
class cache_site_selector: public rcppsw::er::client<cache_site_selector> {
 public:
  struct cache_constraint_data {
    const representation::base_cache* mc_cache{nullptr};
    cache_site_selector*        selector{nullptr};
    double                      cache_prox_dist{0.0};
  };
  struct block_constraint_data {
    const representation::base_block* mc_block{nullptr};
    cache_site_selector*        selector{nullptr};
    double                      block_prox_dist{0.0};
  };
  struct nest_constraint_data {
    rmath::vector2d      nest_loc{};
    cache_site_selector* selector{nullptr};
    double               nest_prox_dist{0.0};
  };
  struct site_utility_data {
    rmath::vector2d position{};
    rmath::vector2d nest_loc{};
  };

  using cache_constraint_vector = std::vector<cache_constraint_data>;
  using block_constraint_vector = std::vector<block_constraint_data>;
  using nest_constraint_vector = std::vector<nest_constraint_data>;

  explicit cache_site_selector(const controller::cache_sel_matrix* matrix);

  ~cache_site_selector(void) override = default;
  cache_site_selector& operator=(const cache_site_selector& other) = delete;
  cache_site_selector(const cache_site_selector& other) = delete;

  /**
   * @brief Given a list of existing caches/blocks that a robot knows about
   * (i.e. have not faded into an unknown state), compute the best site to start
   * a new cache.
   *
   * @return The location of the best cache site, or (-1, -1) if no best cache
   * site could be found (can happen if NLopt mysteriously fails).
   */
  rmath::vector2d calc_best(const ds::dp_cache_map& known_caches,
                            const ds::dp_block_map& known_blocks,
                            rmath::vector2d position);

 private:
  /*
   * @brief The amount of violation of cache constraints that is considered
   * acceptable.
   */
  static constexpr double kCACHE_CONSTRAINT_TOL = 1E-8;

  /*
   * @brief The amount of violation of block constraints that is considered
   * acceptable.
   */
  static constexpr double kBLOCK_CONSTRAINT_TOL = 1E-8;

  /*
   * @brief The amount of violation of nest constraints that is considered
   * acceptable.
   */
  static constexpr double kNEST_CONSTRAINT_TOL = 1E-8;

  /**
   * @brief The difference between utilities evaluated on subsequent timesteps
   * that will be considered indicative of convergence.
   */
  static constexpr double kUTILITY_TOL = 1E-4;

  /**
   * @brief The maximum # of iterations that the optimizer will run. Needed so
   * that it does not bring the simulation to a halt while it chugs and
   * chugs. We *should* be able to get something good enough in this many
   * iterations.
   */
  static constexpr uint kMAX_ITERATIONS = 100000;

  using constraint_set = std::tuple<cache_constraint_vector,
                                    block_constraint_vector,
                                    nest_constraint_vector>;

  /**
   * @brief Create constraints for known caches, known blocks, and relating to
   * the nest.
   */
  void constraints_create(const ds::dp_cache_map& known_caches,
                          const ds::dp_block_map& known_blocks,
                          const rmath::vector2d& nest_loc);


  void opt_initialize(const ds::dp_cache_map& known_caches,
                      const ds::dp_block_map& known_blocks,
                      rmath::vector2d position,
                      struct site_utility_data* utility_data,
                      std::vector<double>* initial_guess);

  bool verify_site(const rmath::vector2d& site,
                   const ds::dp_cache_map& known_caches,
                   const ds::dp_block_map& known_blocks) const;

  /* clang-format off */
  const controller::cache_sel_matrix* const mc_matrix;
  nlopt::opt     m_alg{nlopt::algorithm::GN_ORIG_DIRECT, 2};
  constraint_set m_constraints{};
  /* clang-format on */
};

double __cache_constraint_func(const std::vector<double>& x,
                               std::vector<double>& ,
                               void *data);

double __block_constraint_func(const std::vector<double>& x,
                               std::vector<double>& ,
                               void *data);
double __nest_constraint_func(const std::vector<double>& x,
                               std::vector<double>& ,
                               void *data);

double __site_utility_func(const std::vector<double>& x,
                           std::vector<double>& ,
                           void *data);

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH2_CACHE_SITE_SELECTOR_HPP_ */
