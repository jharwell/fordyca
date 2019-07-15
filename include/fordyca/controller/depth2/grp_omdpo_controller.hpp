/**
 * @file grp_omdpo_controller.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GRP_OMDPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GRP_OMDPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
class oracular_info_receptor;
NS_START(depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class grp_omdpo_controller
 * @ingroup fordyca controller depth2
 *
 * @brief A foraging controller built on \ref grp_mdpo_controller that has
 * perfect information about one or more of the following, depending on
 * configuration:
 *
 * - Block/cache locations
 * - Task duration/estimates
 */
class grp_omdpo_controller : public depth2::grp_mdpo_controller,
                            public rer::client<grp_omdpo_controller> {
 public:
  grp_omdpo_controller(void) RCSW_COLD;
  ~grp_omdpo_controller(void) override RCSW_COLD;

  /* CCI_Controller overrides */
  void ControlStep(void) override;

  std::type_index type_index(void) const override { return {typeid(*this)}; }

  void oracle_init(std::unique_ptr<oracular_info_receptor> receptor) RCSW_COLD;

 private:
  /* clang-format off */
  std::unique_ptr<oracular_info_receptor> m_receptor;
  /* clang-format on */
};

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GRP_OMDPO_CONTROLLER_HPP_ */
