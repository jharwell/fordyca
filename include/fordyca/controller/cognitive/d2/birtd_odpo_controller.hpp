/**
 * \file birtd_odpo_controller.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class birtd_odpo_controller
 * \ingroup controller cognitive d2
 *
 * \brief A foraging controller built on \ref birtd_dpo_controller that has
 * perfect information about one or more of the following, depending on
 * configuration:
 *
 * - Block/cache locations
 * - Task duration/estimates
 */
class birtd_odpo_controller : public d2::birtd_dpo_controller,
                              public rer::client<birtd_odpo_controller> {
 public:
  birtd_odpo_controller(void) RCPPSW_COLD;
  ~birtd_odpo_controller(void) override RCPPSW_COLD;

  /* foraging_controller overrides */
  void control_step(void) override;
  std::type_index type_index(void) const override { return {typeid(*this)}; }

  void oracle_init(std::unique_ptr<fsperception::oracular_info_receptor> receptor) RCPPSW_COLD;

 private:
  /* clang-format off */
  std::unique_ptr<fsperception::oracular_info_receptor> m_receptor;
  /* clang-format on */
};

NS_END(cognitive, d2, controller, fordyca);

