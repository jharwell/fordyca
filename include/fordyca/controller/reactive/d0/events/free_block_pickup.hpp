/**
 * \file free_block_pickup.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/controller/operations/base_block_pickup.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, reactive, d0, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class free_block_pickup
 * \ingroup controller reactive d0 events
 *
 * \brief Fired whenever a robot picks up a free block in the arena.
 */
class free_block_pickup : public rer::client<free_block_pickup>,
                          public ccops::base_block_pickup {
 private:
  struct visit_typelist_impl {
    using controllers = fcontroller::d0::reactive_typelist;
    using fsms = rmpl::typelist<ffsm::d0::crw_fsm>;
    using value = boost::mpl::joint_view<controllers, fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~free_block_pickup(void) override = default;

  free_block_pickup(const free_block_pickup&) = delete;
  free_block_pickup& operator=(const free_block_pickup&) = delete;

  /* CRW foraging */
  void visit(fcreactive::d0::crw_controller& controller);

  free_block_pickup(crepr::base_block3D* block,
                    const rtypes::type_uuid& id,
                    const rtypes::timestep& t);

 private:
  using ccops::base_block_pickup::visit;

  void visit(ffsm::d0::crw_fsm& fsm);
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using free_block_pickup_visitor = rpvisitor::filtered_visitor<free_block_pickup>;

NS_END(events, d0, reactive, controller, fordyca);

