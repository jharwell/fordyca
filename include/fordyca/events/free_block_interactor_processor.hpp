/**
 * \file free_block_interactor_processor.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
