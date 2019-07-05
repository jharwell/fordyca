/**
 * @file depth1_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_DEPTH1_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_DEPTH1_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <memory>

#include "fordyca/support/depth0/depth0_loop_functions.hpp"
#include "fordyca/support/robot_los_updater.hpp"
#include "fordyca/support/robot_metric_extractor.hpp"
#include "fordyca/support/robot_task_extractor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace config { namespace caches { struct caches_config; }}

NS_START(support, depth1);
class depth1_metrics_aggregator;
class static_cache_manager;

template<typename T>
class robot_arena_interactor;

namespace detail {
struct functor_maps_initializer;
template<typename T>
struct d1_subtask_status_extractor;
using d1_subtask_status_map_type =
    rds::type_map<rmpl::typelist_wrap_apply<controller::depth1::typelist,
                                            d1_subtask_status_extractor>::type>;

} /* namespace detail */

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class loop_functions
 * @ingroup fordyca support depth1
 *
 * @brief The loop functions for depth 1 foraging.
 *
 * Handles all operations robots perform relating to depth 1 foraging and
 * potential usage of static caches (pickup, drop, etc.), along with the depth0
 * operations relating to blocks.
 */
class depth1_loop_functions : public depth0::depth0_loop_functions,
                              public rer::client<depth1_loop_functions> {
 public:
  depth1_loop_functions(void);
  ~depth1_loop_functions(void) override;

  void Init(ticpp::Element& node) override;
  void PreStep() override;
  void Reset(void) override;
  void Destroy(void) override;

 protected:
  /**
   * @brief Initialize depth1 support to be shared with derived classes:
   *
   * - All depth0 shared initialization
   * - Depth1 metric collection
   * - Enable task distribution entropy calculations
   * - Tasking oracle
   */
  void shared_init(ticpp::Element& node);

 private:
  using interactor_map_type = rds::type_map<
   rmpl::typelist_wrap_apply<controller::depth1::typelist,
                               robot_arena_interactor>::type>;
  using los_updater_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::depth1::typelist,
                                robot_los_updater>::type>;
  using task_extractor_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::depth1::typelist,
                                robot_task_extractor>::type>;
  using metric_extractor_typelist = rmpl::typelist<
    robot_metric_extractor<depth1_metrics_aggregator,
                           controller::depth1::gp_dpo_controller>,
    robot_metric_extractor<depth1_metrics_aggregator,
                           controller::depth1::gp_odpo_controller>,
    robot_metric_extractor<depth1_metrics_aggregator,
                           controller::depth1::gp_mdpo_controller>,
    robot_metric_extractor<depth1_metrics_aggregator,
                           controller::depth1::gp_omdpo_controller>
    >;
  using metric_extractor_map_type = rds::type_map<metric_extractor_typelist>;

  /*
   * These are friend classes because they are basically just pieces of the loop
   * functions pulled out for increased clarity/modularity, and are not meant to
   * be used in other contexts. Doing things this way rather than passing 8
   * parameters to the functors seemed much cleaner.
   */
  friend detail::functor_maps_initializer;

  /**
   * @brief Initialize depth0 support not shared with derived classes:
   *
   * - Robot interactions with arena
   * - Various maps mapping controller types to metric collection, controller
   *   initialization, and arena interaction maps (reflection basically).
   * - Static cache handling/management.
   * - Tasking oracle.
   */
  void private_init(void);

  /**
   * @brief Initialize static cache handling/management:
   */
  void cache_handling_init(const config::caches::caches_config *cachep,
                           const config::arena::block_dist_config* distp);

  /**
   * @brief Map the block distribution type to the locations of one or more
   * static caches that will be maintained by the simulation during
   * initialization.
   */
  std::vector<rmath::vector2d> calc_cache_locs(
      const config::arena::block_dist_config* distp);

  /**
   * @brief Initialize all oracles.
   */
  void oracle_init(void);

  /**
   * @brief Process a single robot on a timestep:
   *
   * - Collect metrics from it.
   * - Set its new position, time, LOS from ARGoS.
   * - Have it interact with the environment.
   */
  void robot_timestep_process(argos::CFootBotEntity& robot);

  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;

  /**
   * @brief Count the # of free blocks in the arena.
   */
  uint n_free_blocks(void) const;

  /**
   * @brief Extract the numerical ID of the task each robot is currently
   * executing for use in convergence calculations.
   *
   * @param uint Unused.
   */
  std::vector<int> robot_tasks_extract(uint) const;

  /**
   * @brief Monitor the status of the static cache(s), calculating respawn
   * probability and potentially recreating depleted caches as needed.
   *
   */
  void static_cache_monitor(void);

  /* clang-format off */
  std::unique_ptr<interactor_map_type>                m_interactor_map;
  std::unique_ptr<metric_extractor_map_type>          m_metric_extractor_map;
  std::unique_ptr<los_updater_map_type>               m_los_update_map;
  std::unique_ptr<task_extractor_map_type>            m_task_extractor_map;
  std::unique_ptr<detail::d1_subtask_status_map_type> m_subtask_status_map;

  std::unique_ptr<depth1_metrics_aggregator>          m_metrics_agg;
  std::unique_ptr<static_cache_manager>               m_cache_manager;
  /* clang-format on */
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_DEPTH1_LOOP_FUNCTIONS_HPP_ */
