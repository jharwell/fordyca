/**
 * \file cache_found.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"

#include "fordyca/controller/controller_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {
class base_cache;
}

NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_found
 * \ingroup controller cognitive d2 events
 *
 * \brief Created whenever a NEW cache (i.e. one that is not currently known to
 * a robot, but possibly one that it has seen before and whose relevance had
 * expired) is discovered by the robot via it appearing in the robot's LOS.
 */
class cache_found : public rer::client<cache_found> {
 private:
  struct visit_typelist_impl {
    using value = fcontroller::d2::cognitive_typelist::type;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit cache_found(carepr::base_cache* cache);
  ~cache_found(void) override = default;

  cache_found(const cache_found& op) = delete;
  cache_found& operator=(const cache_found& op) = delete;

  /* controllers */
  void visit(fccd2::birtd_dpo_controller& c);
  void visit(fccd2::birtd_mdpo_controller& c);
  void visit(fccd2::birtd_odpo_controller& c);
  void visit(fccd2::birtd_omdpo_controller& c);

 private:
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

NS_END(events, d2, cognitive, controller, fordyca);

