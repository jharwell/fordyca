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
#include <utility>
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
class cache_selection_matrix;
NS_START(depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_site_selector
 * @ingroup depth2
 *
 * @brief Selects the best cache site between the location of the block pickup
 * and the nest (ideally the halfway point), subject to constraints such as it
 * can't be too near other known caches.
 */
class cache_site_selector: public rcppsw::er::client<cache_site_selector> {
 public:
  struct cache_constraint_data {
    representation::base_cache* cache;
    double cache_prox_dist;
  };
  struct block_constraint_data {
    representation::base_block* block;
    double block_prox_dist;
  };
  struct site_utility_data {
    argos::CVector2 robot_loc;
    argos::CVector2 nest_loc;
  };

  using cache_list = std::list<std::shared_ptr<representation::base_cache>>;
  using cache_constraint_vector = std::vector<cache_constraint_data>;
  using block_list = std::list<std::shared_ptr<representation::base_block>>;
  using block_constraint_vector = std::vector<block_constraint_data>;

  explicit cache_site_selector(const controller::cache_selection_matrix* matrix);

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

 private:
  static constexpr double kCACHE_CONSTRAINT_TOL = 1E-8;
  static constexpr double kBLOCK_CONSTRAINT_TOL = 1E-8;
  static constexpr double kUTILITY_TOL = 1E-4;

  using constraint_set = std::pair<cache_constraint_vector,
                                   block_constraint_vector>;
  void constraints_create(const cache_list& known_caches,
                          const block_list& known_blocks,
                          constraint_set* constraints);

  // clang-format off
  const controller::cache_selection_matrix* const mc_matrix;
  nlopt::opt*                                     m_alg{nullptr};
  // clang-format on
};

double __cache_constraint_func(const std::vector<double>& x,
                               std::vector<double>& ,
                               void *data);

double __block_constraint_func(const std::vector<double>& x,
                               std::vector<double>& ,
                               void *data);

double __site_utility_func(const std::vector<double>& x,
                           std::vector<double>& ,
                           void *data);

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH2_CACHE_SITE_SELECTOR_HPP_ */
