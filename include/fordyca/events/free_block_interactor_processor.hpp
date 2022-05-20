/**
 * \file free_block_interactor_processor.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/polled_task.hpp"

#include "fordyca/events/free_block_interactor.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/tasks/base_foraging_task.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class free_block_interactor_processor
 * \ingroup events
 *
 * \brief
 */
class free_block_interactor_processor
    : public rer::client<free_block_interactor_processor> {
 public:
  free_block_interactor_processor(void)
      : ER_CLIENT_INIT("fordyca.events.free_block_interactor_processor") {}

  /* Not move/copy constructable/assignable by default */
  free_block_interactor_processor(const free_block_interactor_processor&) = delete;
  free_block_interactor_processor&
  operator=(const free_block_interactor_processor&) = delete;
  free_block_interactor_processor(free_block_interactor_processor&&) = delete;
  free_block_interactor_processor&
  operator=(free_block_interactor_processor&&) = delete;

  template <typename TVisitor>
  void task_dispatch(tasks::base_foraging_task* const task, TVisitor& visitor) {
    auto* polled RCPPSW_UNUSED = dynamic_cast<cta::polled_task*>(task);
    auto* interactor = dynamic_cast<fevents::free_block_interactor*>(task);
    ER_ASSERT(nullptr != interactor,
              "Non free block interactor task %s causing free block pickup",
              polled->name().c_str());
    interactor->accept(visitor);
  }
};

NS_END(events, fordyca);
