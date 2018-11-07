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

#include <argos3/core/utility/math/vector2.h>
#include <nlopt.hpp>

#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace representation {
class base_cache;
class base_block;
}
NS_START(controller);
class cache_sel_matrix;
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
    representation::base_cache* cache{nullptr};
    cache_site_selector*        selector{nullptr};
    double                      cache_prox_dist{0.0};
  };
  struct block_constraint_data {
    representation::base_block* block{nullptr};
    cache_site_selector*        selector{nullptr};
    double                      block_prox_dist{0.0};
  };
  struct nest_constraint_data {
    argos::CVector2      nest_loc{};
    cache_site_selector* selector{nullptr};
    double               nest_prox_dist{0.0};
  };
  struct site_utility_data {
    argos::CVector2 robot_loc{};
    argos::CVector2 nest_loc{};
  };

  using cache_list = std::list<std::shared_ptr<representation::base_cache>>;
  using cache_constraint_vector = std::vector<cache_constraint_data>;
  using block_list = std::list<std::shared_ptr<representation::base_block>>;
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
   * @return The local of the best cache site.
   */
  argos::CVector2 calc_best(const cache_list& known_caches,
                            const block_list& known_blocks,
                            argos::CVector2 robot_loc);

  void cc_violated(uint id) { m_cc_violations[id] = true; }
  void cc_satisfied(uint id) { m_cc_violations[id] = false; }
  void bc_violated(uint id) { m_bc_violations[id] = true; }
  void bc_satisfied(uint id) { m_bc_violations[id] = false; }
  void nc_violated(void) { m_nc_violations = true; }
  void nc_satisfied(void) { m_nc_violations = false; }

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
  static constexpr uint kMAX_ITERATIONS = 10000;

  using constraint_set = std::tuple<cache_constraint_vector,
                                    block_constraint_vector,
                                    nest_constraint_vector>;

  /**
   * @brief Create constraints for known caches, known blocks, and relating to
   * the nest.
   */
  void constraints_create(const cache_list& known_caches,
                          const block_list& known_blocks,
                          const argos::CVector2& nest_loc,
                          constraint_set* constraints);


  void opt_initialize(const cache_list& known_caches,
                      const block_list& known_blocks,
                      argos::CVector2 robot_loc,
                      constraint_set* constraints,
                      struct site_utility_data* utility_data,
                      std::vector<double>* initial_guess);

  // clang-format off
  const controller::cache_sel_matrix* const mc_matrix;
  nlopt::opt                                      m_alg{nlopt::algorithm::GN_ORIG_DIRECT, 2};
  std::vector<bool> m_cc_violations{};
  std::vector<bool> m_bc_violations{};
  bool              m_nc_violations{0};
  // clang-format on
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
