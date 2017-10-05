/**
 * @file task_arguments.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_TASK_ARGUMENTS_HPP_
#define INCLUDE_FORDYCA_FSM_TASK_ARGUMENTS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos/core/utility/math/vector2.h>
#include "rcppsw/task_allocation/taskable.hpp"
#include "fordyca/controller/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace task_allocation = rcppsw::task_allocation;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class vector_argument : public task_allocation::taskable_argument {
 public:
  explicit vector_argument(const argos::CVector2& v) : m_vector(v) {}

  const argos::CVector2& vector(void) const { return m_vector; }

 private:
  argos::CVector2 m_vector;
};

class foraging_signal_argument : public task_allocation::taskable_argument {
 public:
  explicit foraging_signal_argument(controller::foraging_signal::type s) :
      m_signal(s) {}

  controller::foraging_signal::type signal(void) const { return m_signal; }

 private:
  controller::foraging_signal::type m_signal;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_TASK_ARGUMENTS_HPP_ */
