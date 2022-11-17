/**
 * \file task_abort_interactor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>

#include "cosm/interactors/base_task_abort.hpp"

#include "fordyca/tasks/task_status.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class task_abort_interactor
 * \ingroup argos support
 *
 * \brief Handles a robot's (possible) aborting of its current task on a given
 * timestep.
 */
template <typename TController, typename TControllerSpecMap>
class task_abort_interactor
    : public cinteractors::base_task_abort<TController, TControllerSpecMap> {
 public:
  using
      typename cinteractors::base_task_abort<TController,
                                             TControllerSpecMap>::arena_map_type;
  using typename cinteractors::base_task_abort<TController,
                                               TControllerSpecMap>::envd_type;

  task_abort_interactor(arena_map_type* const map,
                        envd_type* const envd,
                        ::argos::CFloorEntity* const floor)
      : cinteractors::base_task_abort<TController, TControllerSpecMap>(map,
                                                                       envd,
                                                                       floor) {}

  task_abort_interactor(task_abort_interactor&&) = default;

  /* not copy assignable/constructible by default */
  task_abort_interactor(const task_abort_interactor&) = delete;
  task_abort_interactor& operator=(const task_abort_interactor&) = delete;

  bool robot_task_aborted(const TController& controller) const override {
    return (nullptr != controller.current_task() &&
            tasks::task_status::ekABORT_PENDING == controller.task_status());
  }
};

NS_END(support, argos, fordyca);
