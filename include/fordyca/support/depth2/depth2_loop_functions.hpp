/**
 * \file depth2_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <memory>

#include "fordyca/support/depth1/depth1_loop_functions.hpp"
#include "fordyca/support/robot_task_extractor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
class depth2_metrics_aggregator;
class dynamic_cache_manager;
template<typename T>
class robot_arena_interactor;

namespace detail {
struct functor_maps_initializer;
} /* namespace detail */

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class depth2_loop_functions
 * \ingroup support depth2
 *
 * \brief The loop functions for depth 2 foraging.
 *
 * Handles all operations robots perform relating to dynamic caches: pickup,
 * drop, creation, depletion, etc.
 */
class depth2_loop_functions final : public depth1::depth1_loop_functions,
                                    public rer::client<depth2_loop_functions> {
 public:
  depth2_loop_functions(void) RCSW_COLD;
  ~depth2_loop_functions(void) override RCSW_COLD;

  /* swarm manager overrides */
  void init(ticpp::Element& node) override RCSW_COLD;
  void pre_step() override;
  void post_step() override;
  void reset(void) override RCSW_COLD;
  void destroy(void) override RCSW_COLD;

  /**
   * \brief Initialize depth2 support to be shared with derived classes
   *
   * - All depth1 shared initialization
   * - Depth2 metric collection
   */
  void shared_init(ticpp::Element& node) RCSW_COLD;

 private:
  using interactor_map_type = rds::type_map<
   rmpl::typelist_wrap_apply<controller::depth2::typelist,
                               robot_arena_interactor>::type>;
  using los_updater_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::depth2::typelist,
                              cfops::robot_los_update>::type>;
  using task_extractor_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::depth2::typelist,
                                robot_task_extractor>::type>;

  using metric_extractor_typelist = rmpl::typelist<
    ccops::metrics_extract<controller::depth2::birtd_dpo_controller,
                           depth2_metrics_aggregator>,
    ccops::metrics_extract<controller::depth2::birtd_odpo_controller,
                           depth2_metrics_aggregator>,
    ccops::metrics_extract<controller::depth2::birtd_mdpo_controller,
                           depth2_metrics_aggregator>,
    ccops::metrics_extract<controller::depth2::birtd_omdpo_controller,
                           depth2_metrics_aggregator>
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

  void private_init(void) RCSW_COLD;

  void cache_handling_init(const config::caches::caches_config* cachep) RCSW_COLD;

  /**
   * \brief Handle creation of dynamic caches during initialization, reset, or
   * when triggered by events during simulation.
   *3a
   * \param on_drop \c TRUE if caches are to be (potentially) created as a
   * result of a robot block drop. If \c FALSE, then consider dynamic cache
   * creation in other situations.
   *
   * \return \c TRUE if one or more caches were created, \c FALSE otherwise.
   */
  bool cache_creation_handle(bool on_drop);

  /**
   * \brief Extract the numerical ID of the task each robot is currently
   * executing for use in convergence calculations.
   *
   * Cannot use depth1 version as the binary layout of the controllers is not
   * guaranteed to be the same, AND the task mappers will throw key errors if
   * you try it.
   *
   * \param uint Unused.
   */
  std::vector<int> robot_tasks_extract(uint) const;

    /**
   * \brief Process a single robot on a timestep, before running its controller:
   *
   * - Set its new position, time, LOS from ARGoS.
   */
  void robot_pre_step(argos::CFootBotEntity& robot);

  /**
   * \brief Process a single robot on a timestep, after running its controller:
   *
   * - Have  it interact with the environment.
   * - Collect metrics from it.
   */
  void robot_post_step(argos::CFootBotEntity& robot);

  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;

  /* clang-format off */
  std::mutex                                 m_dynamic_cache_mtx{};
  bool                                       m_dynamic_cache_create{false};

  std::unique_ptr<depth2_metrics_aggregator> m_metrics_agg;
  std::unique_ptr<dynamic_cache_manager>     m_cache_manager;
  std::unique_ptr<interactor_map_type>       m_interactor_map;
  std::unique_ptr<metric_extractor_map_type> m_metric_extractor_map;
  std::unique_ptr<los_updater_map_type>      m_los_update_map;
  std::unique_ptr<task_extractor_map_type>   m_task_extractor_map;
  /* clang-format on */
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_LOOP_FUNCTIONS_HPP_ */
