/**
 * \file argument.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "cosm/ta/taskable.hpp"
#include "cosm/ta/taskable_argument.hpp"

#include "fordyca/fsm/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_signal_argument
 * \ingroup tasks
 *
 * \brief An argument that can be passed to a \ref
 * cta::taskable::task_start() function which contains a
 * foraging signal, for use in specifying initial conditions/commands for
 * certain state machines.
 */
class foraging_signal_argument : public cta::taskable_argument {
 public:
  explicit foraging_signal_argument(fsm::foraging_signal::type s) : m_signal(s) {}

  fsm::foraging_signal::type signal(void) const { return m_signal; }

 private:
  fsm::foraging_signal::type m_signal;
};

NS_END(tasks, fordyca);
