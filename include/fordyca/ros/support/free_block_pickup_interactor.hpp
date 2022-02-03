/**
 * \file free_block_pickup_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_ROS_SUPPORT_FREE_BLOCK_PICKUP_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_ROS_SUPPORT_FREE_BLOCK_PICKUP_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <boost/mpl/at.hpp>

#include "rcppsw/utils/color.hpp"

#include "cosm/tv/temporal_penalty.hpp"
#include "cosm/repr/real_block3D.hpp"
#include "cosm/controller/operations/base_block_pickup.hpp"

#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/metrics/blocks/block_manip_events.hpp"
#include "fordyca/support/interactor_status.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ros, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class free_block_pickup_interactor
 * \ingroup ros support
 *
 * \brief Handle's a robot's (possible) free block pickup event on a given
 * timestep.
 */
template <typename TController, typename TControllerSpecMap>
class free_block_pickup_interactor final {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;
  using robot_block_pickup_visitor_type =
      typename controller_spec::robot_block_pickup_visitor_type;

  free_block_pickup_interactor(void)
      : ER_CLIENT_INIT("fordyca.ros.support.free_block_interactor") {}

  free_block_pickup_interactor(free_block_pickup_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  free_block_pickup_interactor(const free_block_pickup_interactor&) = delete;
  free_block_pickup_interactor&
  operator=(const free_block_pickup_interactor&) = delete;

  /**
   * \brief Handle robot-arena interactions for the specified controller
   * instance on this timestep.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  fsupport::interactor_status operator()(TController& controller,
                                         const rtypes::timestep& t) {
    if (controller.block_detected()) {
      if (controller.is_carrying_block()) {
        ER_INFO("Ignoring detected block--already carrying block");
      } else {
        execute_pickup(controller, t);
        return fsupport::interactor_status::ekARENA_FREE_BLOCK_PICKUP;
      }
    }
    return fsupport::interactor_status::ekNO_EVENT;
  }

 private:
  /**
   * \brief Perform the actual picking up of a free block once all
   * preconditions have been satisfied.
   */
  void execute_pickup(TController& controller, const rtypes::timestep& t) {
    auto block = std::make_unique<crepr::real_block3D>(rutils::color::kBLACK,
                                                       crepr::block_type::ekNONE);

    robot_block_pickup_visitor_type rpickup_op(block.get(),
                                               controller.entity_id(),
                                               t);

    rpickup_op.visit(controller);
  }
};

NS_END(support, ros, fordyca);

#endif /* INCLUDE_FORDYCA_ROS_SUPPORT_FREE_BLOCK_PICKUP_INTERACTOR_HPP_ */
