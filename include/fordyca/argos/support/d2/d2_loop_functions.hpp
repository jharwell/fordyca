/**
 * \file d2_loop_functions.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <memory>
#include <mutex>

#include "cosm/controller/operations/task_id_extract.hpp"

#include "fordyca/argos/support/d1/d1_loop_functions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca::argos::metrics::d2 {
class d2_metrics_manager;
} /* namespace fordyca::argos::metrics::d1 */

NS_START(fordyca, argos, support, d2);
class dynamic_cache_manager;
template<typename TController, typename TArenaMap>
class robot_arena_interactor;

namespace detail {
struct functor_maps_initializer;
} /* namespace detail */

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class d2_loop_functions
 * \ingroup argos support d2
 *
 * \brief The loop functions for depth 2 foraging.
 *
 * Handles all operations robots perform relating to dynamic caches: pickup,
 * drop, creation, depletion, etc.
 */
class d2_loop_functions final : public d1::d1_loop_functions,
                                    public rer::client<d2_loop_functions> {
 public:
  d2_loop_functions(void) RCPPSW_COLD;
  ~d2_loop_functions(void) override RCPPSW_COLD;

  /* swarm manager overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void pre_step() override;
  void post_step() override;
  void reset(void) override RCPPSW_COLD;
  void destroy(void) override RCPPSW_COLD;

  /**
   * \brief Initialize d2 support to be shared with derived classes
   *
   * - All d1 shared initialization
   * - Depth2 metric collection
   */
  void shared_init(ticpp::Element& node) RCPPSW_COLD;

 private:
  using interactor_map_type = rds::type_map<
   rmpl::typelist_wrap_apply<controller::d2::typelist,
                             robot_arena_interactor,
                             carena::caching_arena_map>::type>;
  using los_updater_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::d2::typelist,
                              ccops::grid_los_update,
                              rds::grid2D_overlay<cds::cell2D>,
                              repr::forager_los>::type>;
  using task_extractor_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::d2::typelist,
                              ccops::task_id_extract>::type>;

  using metric_extractor_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<controller::d2::typelist,
                              ccops::metrics_extract,
                              fametrics::d2::d2_metrics_manager>::type>;

  /**
   * \brief These are friend classes because they are basically just pieces of
   * the loop functions pulled out for increased clarity/modularity, and are not
   * meant to be used in other contexts.
   *
   * Doing things this way rather than passing 8 parameters to the functors
   * seemed much cleaner.
   */
  friend detail::functor_maps_initializer;

  void private_init(void) RCPPSW_COLD;

  void cache_handling_init(const fascaches::config::caches_config* cachep) RCPPSW_COLD;

  /**
   * \brief Handle creation of dynamic caches during initialization, reset, or
   * when triggered by events during simulation.
   *
   * \param on_drop \c TRUE if caches are to be (potentially) created as a
   * result of a robot block drop. If \c FALSE, then consider dynamic cache
   * creation in other situations.
   *
   * \return \c TRUE if one or more caches were created, \c FALSE otherwise.
   */
  bool cache_creation_handle(bool on_drop) RCPPSW_COLD;

  /**
   * \brief Extract the numerical ID of the task each robot is currently
   * executing for use in convergence calculations.
   *
   * Cannot use d1 version as the binary layout of the controllers is not
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
  void robot_pre_step(chal::robot& robot);

  /**
   * \brief Process a single robot on a timestep, after running its controller:
   *
   * - Have  it interact with the environment.
   * - Collect metrics from it.
   */
  void robot_post_step(chal::robot& robot);

  /* clang-format off */
  std::mutex                                         m_dynamic_cache_mtx{};
  bool                                               m_dynamic_cache_create{false};

  std::unique_ptr<fametrics::d2::d2_metrics_manager> m_metrics_manager;
  std::unique_ptr<dynamic_cache_manager>             m_cache_manager;
  std::unique_ptr<interactor_map_type>               m_interactor_map;
  std::unique_ptr<metric_extractor_map_type>         m_metric_extractor_map;
  std::unique_ptr<los_updater_map_type>              m_los_update_map;
  std::unique_ptr<task_extractor_map_type>           m_task_extractor_map;
  /* clang-format on */
};

NS_END(d2, support, argos, fordyca);
