/**
 * \file birtd_odpo_controller.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D2_BIRTD_ODPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D2_BIRTD_ODPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive);
class oracular_info_receptor;
NS_START(d2);

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

  void oracle_init(std::unique_ptr<oracular_info_receptor> receptor) RCPPSW_COLD;

 private:
  /* clang-format off */
  std::unique_ptr<oracular_info_receptor> m_receptor;
  /* clang-format on */
};

NS_END(cognitive, d2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D2_BIRTD_ODPO_CONTROLLER_HPP_ */
