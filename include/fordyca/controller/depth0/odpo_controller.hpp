/**
 * @file odpo_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH0_ODPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH0_ODPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/depth0/dpo_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
class oracular_info_receptor;
NS_START(depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class odpo_controller
 * @ingroup fordyca controller depth0
 *
 * @brief A foraging controller derived from \ref dpo_controller that has
 * perfect information about blocks in the arena.
 */
class odpo_controller : public dpo_controller,
                        public rer::client<odpo_controller> {
 public:
  odpo_controller(void);
  ~odpo_controller(void) override;

  /* CCI_Controller overrides */
  void ControlStep(void) override;

  std::type_index type_index(void) const override {
    return {typeid(*this)};
  }

  void oracle_init(std::unique_ptr<oracular_info_receptor> receptor);

 private:
  /* clang-format off */
  std::unique_ptr<oracular_info_receptor> m_receptor;
  /* clang-format on */
};

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_ODPO_CONTROLLER_HPP_ */
