/**
 * \file d0/robot_los_update_applicator.hpp
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
#ifndef INCLUDE_FORDYCA_SUPPORT_D0_ROBOT_LOS_UPDATE_APPLICATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_D0_ROBOT_LOS_UPDATE_APPLICATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/type_map.hpp"

#include "cosm/controller/operations/grid_los_update.hpp"

#include "fordyca/controller/controller_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ds {
class arena_grid;
} /* namespace cosm::ds */

namespace fordyca::repr {
class forager_los;
} /* namespace fordyca::repr */

NS_START(fordyca, support, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_los_update_applicator
 * \ingroup support d0
 *
 * \brief Wrapping functor to update robot LOS each timestep. Needed for use
 * with boost::static_visitor.
 */
class robot_los_update_applicator {
 public:
  template<typename TController>
  using los_update_op_type = ccops::grid_los_update<TController,
                                                    rds::grid2D_overlay<cds::cell2D>,
                                                    repr::forager_los>;

  explicit robot_los_update_applicator(controller::foraging_controller* const c)
      : controller(c) {}

  /*
   * If the controller is not derived from DPO, then there is no LOS to update.
   */
  template<typename TController,
           RCPPSW_SFINAE_DECLDEF(!std::is_base_of<controller::cognitive::d0::dpo_controller,
                              TController>::value)>
  void operator()(los_update_op_type<TController>& ) const {}

  template<typename TController,
           RCPPSW_SFINAE_DECLDEF(std::is_base_of<controller::cognitive::d0::dpo_controller,
                              TController>::value)>
  void operator()(los_update_op_type<TController>& impl) const {
    impl(dynamic_cast<TController*>(controller));
  }

 private:
  /* clang-format off */
  controller::foraging_controller* const controller;
  /* clang-format on */
};

NS_END(d0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_D0_ROBOT_LOS_UPDATE_APPLICATOR_HPP_ */
