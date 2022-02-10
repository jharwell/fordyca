/**
 * \file bitd_odpo_controller.hpp
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
#include <memory>

#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bitd_odpo_controller
 * \ingroup controller cognitive d1
 *
 * \brief A foraging controller built on \ref bitd_dpo_controller
 * that has perfect information about on or more of the following, depending on
 * configuration:
 *
 * - Block/cache locations
 * - Task durations/estimates
 */
class bitd_odpo_controller : public d1::bitd_dpo_controller,
                             public rer::client<bitd_odpo_controller> {
 public:
  using bitd_dpo_controller::perception;

  bitd_odpo_controller(void) RCPPSW_COLD;
  ~bitd_odpo_controller(void) override RCPPSW_COLD;

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

