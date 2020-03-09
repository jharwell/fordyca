/**
 * \file depth1_loop_functions.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include <atomic>
#include <utility>

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
 * \class depth1_loop_functions
 * \ingroup support depth1
 *
 * \brief The loop functions for depth 1 foraging.
 *
 * Handles all operations robots perform relating to depth 1 foraging and
 * potential usage of static caches (pickup, drop, etc.), along with the depth0
 * operations relating to blocks.
 */
class depth1_loop_functions : public depth0::depth0_loop_functions,
                              public rer::client<depth1_loop_functions> {
 public:
  depth1_loop_functions(void) RCSW_COLD;
  ~depth1_loop_functions(void) override RCSW_COLD;

  /* swarm manager overrides */
  void init(ticpp::Element& node) override RCSW_COLD;
  void pre_step() override;
  void post_step() override;
  void reset(void) override RCSW_COLD;
  void destroy(void) override RCSW_COLD;

 protected:
  /**
   * \brief Initialize depth1 support to be shared with derived classes:
   *
   * - All depth0 shared initialization
   * - Depth1 metric collection
   * - Enable task distribution entropy calculations
   * - Tasking oracle
   */
  void shared_init(ticpp::Element& node) RCSW_COLD;

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
                           controller::depth1::bitd_dpo_controller>,
    robot_metric_extractor<depth1_metrics_aggregator,
                           controller::depth1::bitd_odpo_controller>,
    robot_metric_extractor<depth1_metrics_aggregator,
                           controller::depth1::bitd_mdpo_controller>,
    robot_metric_extractor<depth1_metrics_aggregator,
                           controller::depth1::bitd_omdpo_controller>
    >;
  using metric_extractor_map_type = rds::type_map<metric_extractor_typelist>;

  /**
   * \brief These are friend classes because they are basically just pieces of
   * the loop functions pulled out for increased clarity/modularity, and are not
   * meant to be used in other contexts.
   *
   * Doing things this way rather than passing 8 parameters to the functors
   * seemed much cleaner.
   */
  friend detail::functor_maps_initializer;

  /**
   * \brief Initialize depth0 support not shared with derived classes:
   *
   * - Robot interactions with arena
   * - Various maps mapping controller types to metric collection, controller
   *   initialization, and arena interaction maps (reflection basically).
   * - Static cache handling/management.
   * - Tasking oracle.
   */
  void private_init(void) RCSW_COLD;

  /**
   * \brief Initialize static cache handling/management:
   */
  void cache_handling_init(const config::caches::caches_config *cachep,
                           const cfconfig::block_dist_config* distp) RCSW_COLD;

  /**
   * \brief Map the block distribution type to the locations of one or more
   * static caches that will be maintained by the simulation during
   * initialization.
   */
  std::vector<rmath::vector2d> calc_cache_locs(
      const cfconfig::block_dist_config* distp) RCSW_COLD;

  /**
   * \brief Initialize all oracles.
   */
  void oracle_init(void) RCSW_COLD;

  /**
   * \brief Process a single robot on a timestep, before running its controller:
   *
   * - Set its new position, time from ARGoS and send it its LOS.
   *
   * \note These operations are done in parallel for all robots (lock free).
   */
  void robot_pre_step(argos::CFootBotEntity& robot);

  /**
   * \brief Process a single robot on a timestep, after running its controller.
   *
   * - Have it interact with the environment.
   * - Collect metrics from it.
   *
   * \note These operations are done in parallel for all robots (with mutual
   *       exclusion as needed).
   */
  void robot_post_step(argos::CFootBotEntity& robot);

  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;

  /**
   * \brief Extract the numerical ID of the task each robot is currently
   * executing for use in convergence calculations.
   *
   * \param uint Unused.
   */
  std::vector<int> robot_tasks_extract(uint) const;

  /**
   * \brief Monitor the status of the static cache(s), calculating respawn
   * probability and potentially recreating depleted caches as needed.
   */
  void static_cache_monitor(void);

  /**
   * \brief Return \c TRUE iff one or more static caches have been depleted this
   * timestep (or on previous timesteps and have not been re-created yet, for
   * whatever reason).
   */
  bool caches_depleted(void) const;

  /**
   * \brief Collect task counts from controllers on timesteps one or more static
   * caches have been depleted in order to calculate re-creation probabilities.
   */
  void caches_recreation_task_counts_collect(
      const controller::foraging_controller* const controller);

  /* clang-format off */
  std::unique_ptr<interactor_map_type>                m_interactor_map;
  std::unique_ptr<metric_extractor_map_type>          m_metric_extractor_map;
  std::unique_ptr<los_updater_map_type>               m_los_update_map;
  std::unique_ptr<task_extractor_map_type>            m_task_extractor_map;
  std::unique_ptr<detail::d1_subtask_status_map_type> m_subtask_status_map;

  std::unique_ptr<depth1_metrics_aggregator>          m_metrics_agg;
  std::unique_ptr<static_cache_manager>               m_cache_manager;
  std::pair<std::atomic_uint, std::atomic_uint>       m_cache_counts{0, 0};
  /* clang-format on */
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_DEPTH1_LOOP_FUNCTIONS_HPP_ */
