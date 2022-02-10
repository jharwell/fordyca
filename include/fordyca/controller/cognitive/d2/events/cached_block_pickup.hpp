/**
 * \file cached_block_pickup.hpp
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
#include "fordyca/controller/cognitive/d1/events/cached_block_pickup.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cached_block_pickup
 * \ingroup controller cognitive d2 events
 *
 * \brief Created whenever a robpot picks up a block from a cache.
 *
 * The cache usage penalty, if there is one, is assessed prior to this event
 * being created, at a higher level.
 */
class cached_block_pickup : public rer::client<cached_block_pickup>,
                            public fccd1::events::cached_block_pickup {
 private:
  struct visit_typelist_impl {
    using controllers = fcontroller::d2::cognitive_typelist;
    using others = rmpl::typelist<ftasks::d2::cache_transferer,
                                  ftasks::d2::cache_collector>;

    using value = boost::mpl::joint_view<controllers, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  cached_block_pickup(const carepr::arena_cache* cache,
                      crepr::base_block3D* block,
                      const rtypes::type_uuid& id,
                      const rtypes::timestep& t);
  ~cached_block_pickup(void) override;

  cached_block_pickup(const cached_block_pickup& op) = delete;
  cached_block_pickup&
  operator=(const cached_block_pickup& op) = delete;

  /* controllers */
  void visit(fccd2::birtd_dpo_controller& controller);
  void visit(fccd2::birtd_mdpo_controller& controller);
  void visit(fccd2::birtd_odpo_controller& controller);
  void visit(fccd2::birtd_omdpo_controller& controller);

  /* tasks */
  void visit(ftasks::d2::cache_transferer& task);
  void visit(ftasks::d2::cache_collector& task);

 private:
  bool dispatch_cache_interactor(
      tasks::base_foraging_task* task,
      controller::cognitive::cache_sel_matrix* csel_matrix);
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
NS_START(detail);
using cached_block_pickup_visitor_impl =
    rpvisitor::precise_visitor<cached_block_pickup,
                               cached_block_pickup::visit_typelist>;

NS_END(detail);

class cached_block_pickup_visitor
    : public detail::cached_block_pickup_visitor_impl {
 public:
  using detail::cached_block_pickup_visitor_impl::cached_block_pickup_visitor_impl;
};

NS_END(events, d2, cognitive, controller, fordyca);

