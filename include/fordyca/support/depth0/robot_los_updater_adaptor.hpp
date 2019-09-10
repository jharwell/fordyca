/**
 * @file depth0/robot_los_updater_adaptor.hpp
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
#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_ROBOT_LOS_UPDATER_ADAPTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_ROBOT_LOS_UPDATER_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"
#include "fordyca/controller/controller_fwd.hpp"
#include "rcppsw/ds/type_map.hpp"
#include "fordyca/support/robot_los_updater.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth0, detail);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
using los_updater_map_type = rds::type_map<
  rmpl::typelist_wrap_apply<controller::depth0::typelist,
                            robot_los_updater>::type>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class robot_los_updater_adaptor
 * @ingroup fordyca support depth0
 *
 * @brief Wrapping functor to update robot LOS each timestep. Needed for use
 * with boost::static_visitor.
 */
class robot_los_updater_adaptor {
 public:
  explicit robot_los_updater_adaptor(controller::base_controller* const c)
      : controller(c) {}

  void operator()(robot_los_updater<controller::depth0::crw_controller>& ) const {}

  template<typename ControllerType,
           RCPPSW_SFINAE_FUNC(!std::is_same<ControllerType,
                                 controller::depth0::crw_controller>::value)>
  void operator()(robot_los_updater<ControllerType>& los_updater) const {
    los_updater(dynamic_cast<ControllerType*>(controller));
  }

 private:
  /* clang-format off */
  controller::base_controller* const controller;
  /* clang-format on */
};

NS_END(detail, depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_ROBOT_LOS_UPDATER_ADAPTOR_HPP_ */
