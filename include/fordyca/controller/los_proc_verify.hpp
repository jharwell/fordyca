/**
 * @file los_proc_verify.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_LOS_PROC_VERIFY_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_LOS_PROC_VERIFY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);
namespace ds {
class dpo_store;
class dpo_semantic_map;
} // namespace ds
namespace repr {
class line_of_sight;
} /* namespace repr */
NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class loc_proc_verify
 * @ingroup fordyca controller
 *
 * @brief Verifies that a LOS has been processed properly and that the contents
 * of the LOS is now accurately reflected in a robots perception.
 */
class los_proc_verify : public rer::client<los_proc_verify> {
 public:
  explicit los_proc_verify(const repr::line_of_sight* const c_los)
      : ER_CLIENT_INIT("fordyca.controller.los_proc_verify"), mc_los(c_los) {}

  los_proc_verify(const los_proc_verify& v) = delete;
  los_proc_verify& operator=(const los_proc_verify& v) = delete;

  bool operator()(const ds::dpo_store* c_dpo) const;
  bool operator()(const ds::dpo_semantic_map* c_map) const;

 private:
  /* clang-format off */
  const repr::line_of_sight* const mc_los;
  /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLLER_LOS_PROC_VERIFY_HPP_ */
