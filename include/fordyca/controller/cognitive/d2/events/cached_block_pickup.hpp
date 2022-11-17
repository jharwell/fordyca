/**
 * \file cached_block_pickup.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
    using value = controllers::type;
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
