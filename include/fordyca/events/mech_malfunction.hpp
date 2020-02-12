/**
 * \file mech_malfunction.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_EVENTS_MECH_MALFUNCTION_HPP_
#define INCLUDE_FORDYCA_EVENTS_MECH_MALFUNCTION_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"
#include "fordyca/controller/controller_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::fsm {
class supervisor_fsm;
} /* namespace fsm */

NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class mech_malfunction
 * \ingroup events detail
 *
 * \brief Fired whenever a robot is determined to have mechanically
 * malfunctioned by the loop functions.
 */
class mech_malfunction : public rer::client<mech_malfunction> {
 private:
  struct visit_typelist_impl {
    using fsms = rmpl::typelist<cfsm::supervisor_fsm>;
    using controllers = rmpl::typelist<controller::foraging_controller>;
    using value = boost::mpl::joint_view<controllers::type, fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  mech_malfunction(void) : ER_CLIENT_INIT("fordyca.events.mech_malfunction") {}
  ~mech_malfunction(void) override = default;

  mech_malfunction(const mech_malfunction& op) = delete;
  mech_malfunction& operator=(const mech_malfunction& op) = delete;

  void visit(cfsm::supervisor_fsm& fsm);
  void visit(controller::foraging_controller& controller);

 private:
  /* clang-format off */
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using mech_malfunction_visitor_impl =
    rpvisitor::precise_visitor<detail::mech_malfunction,
                               detail::mech_malfunction::visit_typelist>;

NS_END(detail);

class mech_malfunction_visitor : public detail::mech_malfunction_visitor_impl {
  using detail::mech_malfunction_visitor_impl::mech_malfunction_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_MECH_MALFUNCTION_HPP_ */
