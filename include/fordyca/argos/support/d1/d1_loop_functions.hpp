/**
 * \file d1_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_ARGOS_SUPPORT_D1_D1_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_ARGOS_SUPPORT_D1_D1_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <memory>
#include <atomic>
#include <utility>

#include "cosm/controller/operations/grid_los_update.hpp"
#include "cosm/controller/operations/task_id_extract.hpp"

#include "fordyca/argos/support/d0/d0_loop_functions.hpp"
#include "fordyca/argos/support/caches/config/caches_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::config {
struct block_dist_config;
} /* namespace cosm::foraging::config */

namespace fordyca::argos::metrics::d1 {
class d1_metrics_manager;
} /* namespace fordyca::argos::metrics::d1 */
namespace fordyca::config::caches {
 struct caches_config;
} /* namespace fordyca::config::caches */

NS_START(fordyca, argos, support, d1);

class static_cache_manager;

template<typename TController, typename TArenaMap>
class robot_arena_interactor;

namespace detail {
struct functor_maps_initializer;
template<typename T>
struct d1_subtask_status_extractor;
using d1_subtask_status_map_type =
    rds::type_map<rmpl::typelist_wrap_apply<controller::d1::typelist,
                                            d1_subtask_status_extractor>::type>;

} /* namespace detail */

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class d1_loop_functions
 * \ingroup argos support d1
 *
 * \brief The loop functions for depth 1 foraging.
 *
 * Handles all operations robots perform relating to depth 1 foraging and
 * potential usage of static caches (pickup, drop, etc.), along with the d0
 * operations relating to blocks.
 */
class d1_loop_functions : public d0::d0_loop_functions,
                          public rer::client<d1_loop_functions> {
 public:
  d1_loop_functions(void) RCPPSW_COLD;
  ~d1_loop_functions(void) override RCPPSW_COLD;

  /* swarm manager overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void pre_step() override;
  void post_step() override;
  void reset(void) override RCPPSW_COLD;
  void destroy(void) override RCPPSW_COLD;

 protected:
  /**
   * \brief Initialize d1 support to be shared with derived classes:
   *
   * - All d0 shared initialization
   * - Depth1 metric collection
   * - Enable task distribution entropy calculations
   * - Tasking oracle
   */
  void shared_init(ticpp::Element& node) RCPPSW_COLD;

 private:
  struct cache_counts {
    std::atomic_uint n_harvesters{0};
    std::atomic_uint n_collectors{0};
  };

  using interactor_map_type = rds::type_map<
   rmpl::typelist_wrap_apply<controller::d1::typelist,
                             robot_arena_interactor,
                             carena::caching_arena_map>::type>;
  using los_updater_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::d1::typelist,
                              ccops::grid_los_update,
                              rds::grid2D_overlay<cds::cell2D>,
                              repr::forager_los>::type>;
  using task_extractor_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::d1::typelist,
                              ccops::task_id_extract>::type>;
  using metric_extractor_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::d1::typelist,
                              ccops::metrics_extract,
                              fametrics::d1::d1_metrics_manager>::type>;

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
   * \brief Initialize d0 support not shared with derived classes:
   *
   * - Robot interactions with arena
   * - Various maps mapping controller types to metric collection, controller
   *   initialization, and arena interaction maps (reflection basically).
   * - Static cache handling/management.
   * - Tasking oracle.
   */
  void private_init(void) RCPPSW_COLD;

  /**
   * \brief Initialize static cache handling/management:
   */
  void cache_handling_init(const fascaches::config::caches_config *cachep,
                           const cfconfig::block_dist_config* distp) RCPPSW_COLD;

  /**
   * \brief Initialize all oracles.
   */
  void oracle_init(void) RCPPSW_COLD;

  /**
   * \brief Process a single robot on a timestep, before running its controller:
   *
   * - Set its new position, time from ARGoS and send it its LOS.
   *
   * \note These operations are done in parallel for all robots (lock free).
   */
  void robot_pre_step(chal::robot& robot);

  /**
   * \brief Process a single robot on a timestep, after running its controller.
   *
   * - Have it interact with the environment.
   * - Collect metrics from it.
   *
   * \note These operations are done in parallel for all robots (with mutual
   *       exclusion as needed).
   */
  void robot_post_step(chal::robot& robot);

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
  bool caches_depleted(void) const RCPPSW_PURE;

  /**
   * \brief Collect task counts from controllers on timesteps one or more static
   * caches have been depleted in order to calculate re-creation probabilities.
   */
  void caches_recreation_task_counts_collect(
      const controller::foraging_controller* controller);

  /* clang-format off */
  std::unique_ptr<interactor_map_type>                m_interactor_map;
  std::unique_ptr<metric_extractor_map_type>          m_metric_extractor_map;
  std::unique_ptr<los_updater_map_type>               m_los_update_map;
  std::unique_ptr<task_extractor_map_type>            m_task_extractor_map;
  std::unique_ptr<detail::d1_subtask_status_map_type> m_subtask_status_map;

  std::unique_ptr<fametrics::d1::d1_metrics_manager>  m_metrics_manager;
  std::unique_ptr<static_cache_manager>               m_cache_manager;
  cache_counts                                        m_cache_counts{};
  /* clang-format on */
};

NS_END(d1, support, argos, fordyca);

#endif /* INCLUDE_FORDYCA_ARGOS_SUPPORT_D1_D1_LOOP_FUNCTIONS_HPP_ */
