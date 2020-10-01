/**
 * \file task_abort_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TASK_ABORT_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TASK_ABORT_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>

#include "cosm/interactors/base_task_abort.hpp"

#include "fordyca/tasks/task_status.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class task_abort_interactor
 * \ingroup support
 *
 * \brief Handles a robot's (possible) aborting of its current task on a given
 * timestep.
 */
template <typename TController, typename TControllerSpecMap>
class task_abort_interactor :
    public cinteractors::base_task_abort<TController, TControllerSpecMap> {
 public:
  using typename cinteractors::base_task_abort<TController,
                                               TControllerSpecMap>::arena_map_type;
  using typename cinteractors::base_task_abort<TController,
                                               TControllerSpecMap>::envd_type;

  task_abort_interactor(arena_map_type* const map,
                        envd_type* const envd,
                        argos::CFloorEntity* const floor)
      : cinteractors::base_task_abort<TController, TControllerSpecMap>(map,
                                                                       envd,
                                                                       floor) {}


  task_abort_interactor(task_abort_interactor&&) = default;

  /* not copy assignable/constructible by default */
  task_abort_interactor(const task_abort_interactor& ) = delete;
  task_abort_interactor& operator=(const task_abort_interactor&) = delete;

  bool robot_task_aborted(const TController& controller) const override {
    return (nullptr != controller.current_task() &&
            tasks::task_status::ekABORT_PENDING == controller.task_status());
    }
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TASK_ABORT_INTERACTOR_HPP_ */
