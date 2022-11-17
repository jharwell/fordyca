/**
 * \file task_status.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
