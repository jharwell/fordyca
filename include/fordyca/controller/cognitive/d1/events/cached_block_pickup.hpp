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

#ifndef INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D1_EVENTS_CACHED_BLOCK_PICKUP_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D1_EVENTS_CACHED_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/controller/operations/base_block_pickup.hpp"
#include "cosm/ds/operations/cell2D_op.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/events/block_pickup_base_visit_set.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {
class arena_cache;
} // namespace cosm::arena::repr

namespace fordyca::controller::cognitive {
class cache_sel_matrix;
}

NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cached_block_pickup
 * \ingroup controller cognitive d1 events
 *
 * \brief Created whenever a robpot picks up a block from a cache.
 *
 * The cache usage penalty, if there is one, is assessed prior to this event
 * being created, at a higher level.
 */
class cached_block_pickup : public rer::client<cached_block_pickup>,
                                  public ccops::base_block_pickup {
 private:
  struct visit_typelist_impl {
    using controllers = controller::d1::cognitive_typelist;
    using others = rmpl::typelist<
        fsm::block_to_goal_fsm,
        fsm::d1::cached_block_to_nest_fsm,
      tasks::d1::collector>;

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
  void visit(controller::cognitive::d1::bitd_dpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_mdpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_odpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_omdpo_controller& controller);

  /* tasks */
  void visit(tasks::d1::collector& task);

 protected:
  using ccops::base_block_pickup::visit;

  /* FSMs */
  void visit(fsm::block_to_goal_fsm& fsm);
  void visit(fsm::d1::cached_block_to_nest_fsm& fsm);

  /* data structures */
  void visit(fspds::dpo_semantic_map& map);
  void visit(fspds::dpo_store& store);

  const carepr::arena_cache* cache(void) const { return mc_cache; }

 private:

  /* FSMs */
  void visit(cfsm::cell2D_fsm& fsm);

  /* data structures */
  void visit(cds::cell2D& cell);
  void visit(crepr::base_block3D& block);

  void dispatch_cache_interactor(tasks::base_foraging_task* task);

  /* clang-format off */
  const rtypes::timestep               mc_timestep;
  const carepr::arena_cache*           mc_cache;
  /* clang-format on */
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

NS_END(events, d1, cognitive, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D1_EVENTS_CACHED_BLOCK_PICKUP_HPP_ */
