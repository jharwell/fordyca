/**
 * \file los_proc_verify.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace fordyca::repr {
class forager_los;
} // namespace fordyca::repr

NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class los_proc_verify
 * \ingroup subsystem perception
 *
 * \brief Verifies that a LOS has been processed properly and that the contents
 * of the LOS is now accurately reflected in a robots perception.
 */
class los_proc_verify : public rer::client<los_proc_verify> {
 public:
  explicit los_proc_verify(const repr::forager_los* const c_los)
      : ER_CLIENT_INIT("fordyca.subsystem.perception.los_proc_verify"),
        mc_los(c_los) {}

  los_proc_verify(const los_proc_verify& v) = delete;
  los_proc_verify& operator=(const los_proc_verify& v) = delete;

  bool operator()(const ds::dpo_store* c_dpo) const;
  bool operator()(const ds::dpo_semantic_map* c_map) const;

 private:
  /* clang-format off */
  const repr::forager_los* const mc_los;
  /* clang-format on */
};

NS_END(perception, subsystem, fordyca);

