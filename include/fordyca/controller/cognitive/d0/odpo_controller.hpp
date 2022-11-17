/**
 * \file odpo_controller.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/controller/cognitive/d0/dpo_controller.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class odpo_controller
 * \ingroup controller cognitive d0
 *
 * \brief A foraging controller derived from \ref dpo_controller that has
 * perfect information about blocks in the arena.
 */
class odpo_controller : public dpo_controller,
                        public rer::client<odpo_controller> {
 public:
  odpo_controller(void) RCPPSW_COLD;
  ~odpo_controller(void) override RCPPSW_COLD;

  /* foraging_controller overrides */
  void control_step(void) override;
  std::type_index type_index(void) const override { return typeid(*this); }

  void oracle_init(std::unique_ptr<fsperception::oracular_info_receptor> receptor) RCPPSW_COLD;

 private:
  /* clang-format off */
  std::unique_ptr<fsperception::oracular_info_receptor> m_receptor;
  /* clang-format on */
};

NS_END(cognitive, d0, controller, fordyca);

