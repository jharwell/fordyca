/**
 * \file block_sel_matrix.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <variant>
#include <map>
#include <string>
#include <vector>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "fordyca/controller/config/block_sel/block_sel_matrix_config.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive);

/**
 * \brief \ref boost::variant containing all the different object/POD types that
 * are mapped to within the \ref block_sel_matrix; multiple entries in the
 * matrix can have the same type.
 */
using block_sel_variant =
    std::variant<double,
                   rmath::vector2d,
                   std::vector<rtypes::type_uuid>,
                   config::block_sel::block_pickup_policy_config>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_sel_matrix
 * \ingroup controller cognitive
 *
 * \brief A dictionary of information needed by robots using various utility
 * functions to calculate the best:
 *
 * - block (of whatever type)
 */
class block_sel_matrix : public std::map<std::string, block_sel_variant> {
 public:
  static inline const std::string kNestLoc = "nest_loc";
  static inline const std::string kCubePriority = "cube_priority";
  static inline const std::string kRampPriority = "ramp_priority";
  static inline const std::string kSelExceptions = "sel_exceptions";

  /**
   * \brief The conditions that must be satisfied before a robot will be
   * able to pickup a block (if applicable).
   */
  static inline const std::string kPickupPolicy = "pickup_policy";
  static inline const std::string kPickupPolicyNull = "";
  static inline const std::string kPickupPolicyClusterProx = "cluster_proximity";

  explicit block_sel_matrix(
      const config::block_sel::block_sel_matrix_config* config,
      const rmath::vector2d& nest_loc);

  /**
   * \brief Add a block to the exception list, disqualifying it from being
   * selected as a block to pick up, regardless of its utility value, the next
   * time the robot runs the block selection algorithm.
   *
   * \param id The ID of the block to add.
   */
  void sel_exception_add(const rtypes::type_uuid& id);

  /**
   * \brief Clear the exceptions list. This happens after a robot has executed
   * the task AFTER the task that dropped a free block somewhere in the arena
   * (i.e. there is a 1 task buffer between when a robot drops block X via a
   * free block drop event as part of a task, and when it is allowed to pick
   * that block up again as part of a task).
   */
  void sel_exceptions_clear(void);
};

NS_END(cognitive, controller, fordyca);

