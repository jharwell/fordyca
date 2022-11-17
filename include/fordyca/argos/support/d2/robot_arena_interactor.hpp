/**
 * \file d2/robot_arena_interactor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/d2/cache_site_block_drop_interactor.hpp"
#include "fordyca/argos/support/d2/new_cache_block_drop_interactor.hpp"
#include "fordyca/argos/support/caches/cached_block_pickup_interactor.hpp"
#include "fordyca/argos/support/caches/existing_cache_block_drop_interactor.hpp"
#include "fordyca/argos/support/free_block_pickup_interactor.hpp"
#include "fordyca/argos/support/nest_block_drop_interactor.hpp"
#include "fordyca/support/interactor_status.hpp"
#include "fordyca/tasks/task_status.hpp"
#include "fordyca/argos/support/task_abort_interactor.hpp"
#include "fordyca/argos/support/mpl/free_block_pickup_spec.hpp"
#include "fordyca/argos/support/mpl/nest_block_drop_spec.hpp"
#include "fordyca/support/mpl/free_block_drop_spec.hpp"
#include "fordyca/argos/support/mpl/cached_block_pickup_spec.hpp"
#include "fordyca/argos/support/mpl/cache_block_drop_spec.hpp"
#include "fordyca/argos/support/mpl/task_abort_spec.hpp"
#include "fordyca/controller/cognitive/d2/events/nest_block_drop.hpp"
#include "fordyca/controller/cognitive/d2/events/free_block_pickup.hpp"
#include "fordyca/controller/cognitive/d2/events/block_vanished.hpp"
#include "fordyca/controller/cognitive/d2/events/cache_vanished.hpp"
#include "fordyca/controller/cognitive/d2/events/free_block_drop.hpp"
#include "fordyca/controller/cognitive/d2/events/cache_block_drop.hpp"
#include "fordyca/controller/cognitive/d2/events/cached_block_pickup.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca::argos::metrics::d2 {
class d2_metrics_manager;
} /* namespace fordyca::argos::metrics::d2 */

NS_START(fordyca, argos, support, d2);
class dynamic_cache_manager;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class robot_arena_interactor
 * \ingroup argos support d2
 *
 * \brief Handles a robot's interactions with the environment on each timestep.
 *
 * Including:
 *
 * - Picking up from/dropping a block in a cache.
 * - Subjecting robots using caches to a penalty on cache drop/pickup.
 * - Picking up a free block.
 * - Dropping a carried block in the nest.
 * - Free block drop due to task abort.
 * - Creating a new cache.
 * - Task abort.
 */
template <typename TController, typename TArenaMap>
class robot_arena_interactor final : public rer::client<robot_arena_interactor<TController,
                                                                               TArenaMap>> {
 public:
  using controller_type = TController;

  struct params {
    carena::caching_arena_map* const map;
    fametrics::d2::d2_metrics_manager *const metrics_agg;
    ::argos::CFloorEntity* const floor;
    tv::env_dynamics* const envd;
    dynamic_cache_manager* cache_manager;
    argos_swarm_manager* loop;
  };
  explicit robot_arena_interactor(const params& p)
      : ER_CLIENT_INIT("fordyca.argos.support.d2.robot_arena_interactor"),
        m_free_pickup(p.map,
                      p.floor,
                      p.envd->penalty_handler(tv::block_op_src::ekFREE_PICKUP)),
        m_nest_drop(p.map, p.metrics_agg, p.floor, p.envd),
        m_task_abort(p.map, p.envd, p.floor),
        m_cached_pickup(p.map,
                        p.floor,
                        p.envd,
                        p.cache_manager,
                        p.loop),
        m_existing_cache_drop(p.map, p.envd),
        m_cache_site_drop(p.map,
                          p.floor,
                          p.envd,
                          p.cache_manager),
        m_new_cache_drop(p.map,
                         p.floor,
                         p.envd,
                         p.cache_manager) {}

  robot_arena_interactor(robot_arena_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  robot_arena_interactor(const robot_arena_interactor&) = delete;
  robot_arena_interactor& operator=(const robot_arena_interactor&) = delete;

  /**
   * \brief The actual handling function for interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  fsupport::interactor_status operator()(TController& controller, const rtypes::timestep& t) {
    if (m_task_abort(controller)) {
      /*
       * This needs to be here, rather than in each robot's control step
       * function, in order to avoid triggering erroneous handling of an aborted
       * task in the loop functions when the executive has not aborted the newly
       * allocated task *after* the previous task was aborted. See
       * FORDYCA#532,FORDYCA#587.
       */
      controller.task_status_update(tasks::task_status::ekRUNNING);
      return fsupport::interactor_status::ekTASK_ABORT;
    }

    auto status = fsupport::interactor_status::ekNO_EVENT;
    if (controller.is_carrying_block()) {
      status |= m_nest_drop(controller, t);
      m_existing_cache_drop(controller, t);
      status |= m_cache_site_drop(controller, t);
      status |= m_new_cache_drop(controller, t);
    } else { /* The foot-bot has no block item */
      status |= m_free_pickup(controller, t);
      status |= m_cached_pickup(controller, t);
    }
    return status;
  }


 private:
    using robot_block_vanished_visitor_type = fccd2::events::block_vanished_visitor;
  using robot_free_block_pickup_visitor_type = fccd2::events::free_block_pickup_visitor;
  using robot_nest_block_drop_visitor_type = fccd2::events::nest_block_drop_visitor;
  using robot_free_block_drop_visitor_type = fccd2::events::free_block_drop_visitor;
  using robot_cache_block_drop_visitor_type = fccd2::events::cache_block_drop_visitor;
  using robot_cached_block_pickup_visitor_type = fccd2::events::cached_block_pickup_visitor;
  using robot_cache_vanished_visitor_type = fccd2::events::cache_vanished_visitor;

  using free_pickup_spec = mpl::free_block_pickup_spec<
    controller::d2::typelist,
    robot_block_vanished_visitor_type,
    robot_free_block_pickup_visitor_type
    >;
  using nest_drop_spec = mpl::nest_block_drop_spec<
    controller::d2::typelist,
    robot_nest_block_drop_visitor_type
    >;
  using cached_pickup_spec = mpl::cached_block_pickup_spec<
    controller::d2::typelist,
    robot_cached_block_pickup_visitor_type,
    robot_cache_vanished_visitor_type
    >;
  using cache_drop_spec = mpl::cache_block_drop_spec<
    controller::d2::typelist,
    robot_cache_block_drop_visitor_type,
    robot_cache_vanished_visitor_type
    >;

  using cache_site_drop_spec = fsupport::mpl::free_block_drop_spec<
    controller::d2::typelist,
    robot_free_block_drop_visitor_type
    >;
  using new_cache_drop_spec = fsupport::mpl::free_block_drop_spec<
    controller::d2::typelist,
    robot_free_block_drop_visitor_type
    >;

  using task_abort_spec = fasmpl::task_abort_spec<
    controller::d2::typelist,
    robot_free_block_drop_visitor_type
    >;

  /* clang-format off */
  free_block_pickup_interactor<
    TController,
    free_pickup_spec
    > m_free_pickup;
  nest_block_drop_interactor<
    TController,
    nest_drop_spec
    > m_nest_drop;
  task_abort_interactor<
    TController,
    task_abort_spec
    > m_task_abort;
  caches::cached_block_pickup_interactor<
    TController,
    cached_pickup_spec
    > m_cached_pickup;
  caches::existing_cache_block_drop_interactor<
    TController,
    cache_drop_spec
    > m_existing_cache_drop;
  cache_site_block_drop_interactor<
    TController,
    cache_site_drop_spec
    > m_cache_site_drop;
  new_cache_block_drop_interactor<
    TController,
    new_cache_drop_spec
    > m_new_cache_drop;
  /* clang-format on */
};

NS_END(d2, support, argos, fordyca);

