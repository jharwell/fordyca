/**
 * \file argument.hpp
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

#ifndef INCLUDE_FORDYCA_TASKS_ARGUMENT_HPP_
#define INCLUDE_FORDYCA_TASKS_ARGUMENT_HPP_

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
  explicit foraging_signal_argument(fsm::foraging_signal::type s)
      : m_signal(s) {}

  fsm::foraging_signal::type signal(void) const { return m_signal; }

 private:
  fsm::foraging_signal::type m_signal;
};

NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_ARGUMENT_HPP_ */
