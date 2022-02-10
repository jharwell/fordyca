/**
 * \file task_status.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, tasks);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Enumeration to handle handshaking between robot task abort and arena.
 */
enum class task_status {
  /**
   * \brief Sentinel value indicating a robot is not currently executing a task.
   */
  ekNULL,

  /**
   * \brief A robot is currently executing a task.
   */
  ekRUNNING,

  /**
   * \brief A robot's task executive has aborted the current task, and FORDYCA
   * now needs to perform robot-arena handshaking to update arena state to
   * reflect this.
   */
  ekABORT_PENDING,

  /**
   * \brief Robot-area handshaking as a result of a task abort has been
   * successfully completed.
   */
  ekABORT_PROCESSED
};

NS_END(tasks, fordyca);

