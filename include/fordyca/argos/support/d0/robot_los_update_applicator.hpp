/**
 * \file d0/robot_los_update_applicator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */
#pragma once

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

NS_START(fordyca, argos, support, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_los_update_applicator
 * \ingroup argos support d0
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

NS_END(d0, support, argos, fordyca);

