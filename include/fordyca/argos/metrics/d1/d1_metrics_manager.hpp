/**
 * \file d1_metrics_manager.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/controller/metrics/manipulation_metrics.hpp"
#include "cosm/kin/metrics/kinematics_metrics.hpp"
#include "cosm/spatial/metrics/interference_metrics.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/ta/metrics/bi_tdgraph_metrics.hpp"
#include "cosm/ta/polled_task.hpp"
#include "cosm/metrics/specs.hpp"

#include "fordyca/argos/metrics/d0/d0_metrics_manager.hpp"
#include "fordyca/metrics/perception/dpo_metrics.hpp"
#include "fordyca/metrics/perception/mdpo_metrics.hpp"
#include "fordyca//controller/foraging_controller.hpp"
#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/subsystem/perception/foraging_perception_subsystem.hpp"
#include "fordyca/metrics/specs.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ta::ds {
class bi_tab;
} /* namespace ds */

namespace cosm::arena::repr {
class arena_cache;
} /* namespace cosm::arena */

namespace fordyca::argos::support::caches {
class base_manager;
} /* namespace fordyca::support */

NS_START(fordyca, argos, metrics, d1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class d1_metrics_manager
 * \ingroup argos metrics d1
 *
 * \brief Aggregates and metrics collection for d1 foraging. That
 * includes everything from \ref d0_metrics_manager, and also:
 *
 * - FSM cache acquisition metrics
 * - Cache utilization metrics
 * - Cache lifecycle metrics
 * - World model metrics
 * - Task execution metrics (per task)
 * - TAB metrics (rooted at generalist)
 */
class d1_metrics_manager : public d0::d0_metrics_manager,
                           public rer::client<d1_metrics_manager> {
 public:
  d1_metrics_manager(const rmconfig::metrics_config* mconfig,
                     const cdconfig::grid2D_config* gconfig,
                     const fs::path& output_root,
                     size_t n_block_clusters);

  /**
   * \brief Collect metrics from a finished or aborted task.
   *
   * This cannot be collected synchronously per-timestep with the rest of the
   * metrics from the controller, because by the time metric collecting occurs,
   * the executive has already allocated a new task, and there is not any way to
   * know if a robot's current task is the result of an abort/finish (and is
   * therefore newly allocated and SHOULD have metrics collected from it), or is
   * just running normally.
   *
   * Solution: hook into the executive callback queue in order to correctly
   * capture statistics.
   */
  void task_finish_or_abort_cb(const cta::polled_task* task);

  void task_start_cb(const cta::polled_task*, const cta::ds::bi_tab* tab);

    /**
   * \brief Collect metrics from the d1 controller.
   */
    template<class Controller>
    void collect_from_controller(const Controller* const controller) {
      base_fs_output_manager::collect_from_controller(controller);
      collect_controller_common(controller);
      /*
       * Only controllers with MDPO perception provide these.
       */
      const auto *mdpo = dynamic_cast<const fmetrics::perception::mdpo_metrics*>(
          controller->perception());
      if (nullptr != mdpo) {
        collect(fmspecs::perception::kMDPO.scoped(), *mdpo);
      }
      /*
       * Only controllers with DPO perception provide these.
       */
      const auto *dpo = dynamic_cast<const fmetrics::perception::dpo_metrics*>(
          controller->perception());
      if (nullptr != dpo) {
        collect(fmspecs::perception::kDPO.scoped(), *dpo);
      }
    }

  /**
   * \brief Collect utilization metrics from a cache in the arena.
   */
  void collect_from_cache(const carepr::arena_cache* cache);

  /**
   * \brief Collect lifecycle metrics across all caches in the arena.
   */
  void collect_from_cache_manager(
      const fascaches::base_manager* manager);

 protected:
  /**
   * \brief Register all collectors which require the task decomposition graph
   * depth.
   *
   * This is a protected function so that derived classes aggregating metrics
   * from deeper decomposition graphs can overwrite the original version, and
   * thereby reduce code duplication.
   */
  void register_with_decomp_depth(const rmconfig::metrics_config* mconfig,
                                  size_t depth);
 private:
  template<typename Controller>
  void collect_controller_common(const Controller* const controller) {
    collect(cmspecs::spatial::kNestZone.scoped(), *controller->nz_tracker());
    collect(fmspecs::blocks::kManipulation.scoped(), *controller->block_manip_recorder());

    const auto *task = dynamic_cast<const cta::polled_task*>(controller->current_task());
    if (nullptr == task) {
      return;
    }
    collect(cmspecs::blocks::kTransporter.scoped(), *task->mechanism());
    collect_if(
        cmspecs::blocks::kAcqCounts.scoped(),
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          const auto & m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekBLOCK == m.acquisition_goal();
        });
    collect_if(
        cmspecs::blocks::kAcqLocs2D.scoped(),
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          const auto & m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekBLOCK == m.acquisition_goal() &&
              m.goal_acquired();
        });

    /*
     * We count "false" explorations as part of gathering metrics on where
     * robots explore.
     */
    collect_if(
        cmspecs::blocks::kAcqExploreLocs2D.scoped(),
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          const auto & m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekBLOCK == m.acquisition_goal() &&
              m.is_exploring_for_goal().is_exploring;
        });
    collect_if(
        cmspecs::blocks::kAcqVectorLocs2D.scoped(),
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          const auto & m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekBLOCK == m.acquisition_goal() &&
              m.is_vectoring_to_goal();
        });

    collect_if(
        fmspecs::caches::kAcqCounts.scoped(),
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          const auto & m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekEXISTING_CACHE == m.acquisition_goal();
        });
    collect_if(
        fmspecs::caches::kAcqLocs2D.scoped(),
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          const auto & m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekEXISTING_CACHE == m.acquisition_goal() &&
              m.goal_acquired();
        });

    /*
     * We count "false" explorations as part of gathering metrics on where
     * robots explore.
     */
    collect_if(
        fmspecs::caches::kAcqExploreLocs2D.scoped(),
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          const auto & m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekEXISTING_CACHE == m.acquisition_goal() &&
              m.is_exploring_for_goal().is_exploring;
        });
    collect_if(
        fmspecs::caches::kAcqVectorLocs2D.scoped(),
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          const auto & m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekEXISTING_CACHE == m.acquisition_goal() &&
              m.is_vectoring_to_goal();
        });
    collect(cmspecs::tasks::kDistribution.scoped(), *controller);
  } /* collect_controller_common() */

  void register_standard(const rmconfig::metrics_config* mconfig);

  void register_with_arena_dims2D(const rmconfig::metrics_config* mconfig,
                                  const rmath::vector2z& dims);
};

NS_END(d1, metrics, argos, fordyca);
