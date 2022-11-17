/**
 * \file bitd_omdpo_controller.hpp
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

#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bitd_omdpo_controller
 * \ingroup controller cognitive d1
 *
 * \brief A foraging controller built on \ref bitd_mdpo_controller
 * that has perfect information about on or more of the following, depending on
 * configuration:
 *
 * - Block/cache locations
 * - Task durations/estimates
 */
class bitd_omdpo_controller : public d1::bitd_mdpo_controller,
                            public rer::client<bitd_omdpo_controller> {
 public:
  using bitd_dpo_controller::perception;

  bitd_omdpo_controller(void) RCPPSW_COLD;
  ~bitd_omdpo_controller(void) override RCPPSW_COLD;

  /* foraging_controller overrides */
  void control_step(void) override;
  std::type_index type_index(void) const override { return {typeid(*this)}; }

  void oracle_init(std::unique_ptr<fsperception::oracular_info_receptor> receptor) RCPPSW_COLD;

 private:
  /* clang-format off */
  std::unique_ptr<fsperception::oracular_info_receptor> m_receptor;
  /* clang-format on */
};

NS_END(cognitive, d1, controller, fordyca);

