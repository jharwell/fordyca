/**
 * \file cache_found.hpp
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

#include "cosm/ds/operations/cell2D_op.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {
class base_cache;
}

NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_found
 * \ingroup controller cognitive d1 events
 *
 * \brief Created whenever a NEW cache (i.e. one that is not currently known to
 * a robot, but possibly one that it has seen before and whose relevance had
 * expired) is discovered by the robot via it appearing in the robot's LOS.
 */
class cache_found : public cdops::cell2D_op, public rer::client<cache_found> {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_op::visit_typelist;
    using others = rmpl::typelist<fspds::dpo_store, fspds::dpo_semantic_map>;
    using controllers = controller::d1::typelist;
    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<controllers::type, others::type>,
        inherited::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit cache_found(carepr::base_cache* cache);
  ~cache_found(void) override = default;

  cache_found(const cache_found& op) = delete;
  cache_found& operator=(const cache_found& op) = delete;

  /* controllers */
  void visit(fccd1::bitd_dpo_controller& c);
  void visit(fccd1::bitd_mdpo_controller& c);
  void visit(fccd1::bitd_odpo_controller& c);
  void visit(fccd1::bitd_omdpo_controller& c);

 protected:
  void visit(fspds::dpo_semantic_map& map);
  void visit(fspds::dpo_store& store);

 private:
  /* FSMs */
  void visit(cfsm::cell2D_fsm& fsm);

  /* data structures */
  void visit(cds::cell2D& cell);

  /* clang-format off */
  carepr::base_cache* m_cache;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_found_visitor = rpvisitor::filtered_visitor<cache_found>;

NS_END(events, d1, cognitive, controller, fordyca);

