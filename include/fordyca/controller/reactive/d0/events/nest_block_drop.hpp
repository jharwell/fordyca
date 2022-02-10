/**
 * \file nest_block_drop.hpp
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
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/repr/base_block3D.hpp"

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
 * \class nest_block_drop
 * \ingroup controller reactive d0 events
 *
 * \brief Fired whenever a robot drops a block in the nest.
 */
class nest_block_drop : public rer::client<nest_block_drop> {
 private:
  struct visit_typelist_impl {
    using controllers = fcontroller::d0::reactive_typelist;
    using fsms = rmpl::typelist<ffsm::d0::crw_fsm>;
    using value = boost::mpl::joint_view<controllers, fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  /**
   * \brief Initialize a nest block drop event.
   *
   * \param block The block the robot has just dropped into the nest, owned by
   *              the arena.
   *
   * \param t Current timestep.
   */
  nest_block_drop(crepr::base_block3D* block, const rtypes::timestep& t);

  ~nest_block_drop(void) override = default;

  nest_block_drop(const nest_block_drop& op) = delete;
  nest_block_drop& operator=(const nest_block_drop& op) = delete;

  /* CRW foraging */
  void visit(fcreactive::d0::crw_controller& controller);

 private:
  void visit(ffsm::d0::crw_fsm& fsm);

  /* clang-format off */
  const rtypes::timestep mc_timestep;

  crepr::base_block3D*   m_block;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
NS_START(detail);
using nest_block_drop_visitor_impl =
    rpvisitor::precise_visitor<nest_block_drop,
                               nest_block_drop::visit_typelist>;

NS_END(detail);

class nest_block_drop_visitor : public detail::nest_block_drop_visitor_impl {
 public:
  using detail::nest_block_drop_visitor_impl::nest_block_drop_visitor_impl;
};

NS_END(events, d0, reactive, controller, fordyca);

