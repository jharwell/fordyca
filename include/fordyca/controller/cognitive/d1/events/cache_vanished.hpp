/**
 * \file cache_vanished.hpp
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

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_vanished
 * \ingroup controller cognitive d1 events
 *
 * \brief Created whenever a robot is serving a cache penalty, but while
 * serving the penalty the cache it is waiting in vanishes due to another
 * robot picking up the last available block.
 */
class cache_vanished : public rer::client<cache_vanished> {
 private:
  struct visit_typelist_impl {
    using controllers = fcontroller::d1::cognitive_typelist;
    using tasks = rmpl::typelist<tasks::d1::collector,
                                 tasks::d1::harvester>;
    using fsms = rmpl::typelist<ffsm::block_to_goal_fsm,
                                ffsm::d1::cached_block_to_nest_fsm>;
    using value =
        boost::mpl::joint_view<boost::mpl::joint_view<tasks::type, fsms::type>,
                               controllers::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;
  explicit cache_vanished(const rtypes::type_uuid& cache_id);
  ~cache_vanished(void) override = default;

  cache_vanished(const cache_vanished& op) = delete;
  cache_vanished& operator=(const cache_vanished& op) = delete;

  /* controllers */
  void visit(fccd1::bitd_dpo_controller& controller);
  void visit(fccd1::bitd_mdpo_controller& controller);
  void visit(fccd1::bitd_odpo_controller& controller);
  void visit(fccd1::bitd_omdpo_controller& controller);

  /* tasks */
  void visit(ftasks::d1::collector& task);
  void visit(ftasks::d1::harvester& task);

 protected:
  void visit(ffsm::block_to_goal_fsm& fsm);
  const rtypes::type_uuid& cache_id(void) const { return mc_cache_id; }
  void dispatch_cache_interactor(ftasks::base_foraging_task* task);

 private:
  /* FSMs */
  void visit(ffsm::d1::cached_block_to_nest_fsm& fsm);

  /* clang-format off */
    const rtypes::type_uuid mc_cache_id;
  /* clang-format on */
};


/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_vanished_visitor = rpvisitor::filtered_visitor<cache_vanished>;

NS_END(events, d1, cognitive, controller, fordyca);

