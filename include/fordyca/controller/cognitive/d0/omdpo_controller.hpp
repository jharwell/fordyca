/**
 * \file omdpo_controller.hpp
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

#include "fordyca/controller/cognitive/d0/mdpo_controller.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class omdpo_controller
 * \ingroup controller cognitive d0
 *
 * \brief A foraging controller that has the same properties as \ref
 * mdpo_controller, but that also:
 *
 * - Has perfect information about blocks in the arena
 */
class omdpo_controller : public mdpo_controller,
                         public rer::client<omdpo_controller> {
 public:
  omdpo_controller(void) RCPPSW_COLD;
  ~omdpo_controller(void) override RCPPSW_COLD;

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

