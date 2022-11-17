/**
 * \file block_found.hpp
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

#include "fordyca/fordyca.hpp"
#include "fordyca/controller/controller_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::crepr */

NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_found
 * \ingroup controller cognitive d2 events
 *
 * \brief Event that is created whenever a block (possibly known, possibly
 * unknown) appears in a robot's LOS.
 */
class block_found : public rer::client<block_found> {
 private:
  struct visit_typelist_impl {
    using value = fcontroller::d2::cognitive_typelist;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit block_found(crepr::sim_block3D* block);
  ~block_found(void) override = default;

  block_found(const block_found&) = delete;
  block_found& operator=(const block_found&) = delete;

  /* controllers */
  void visit(fccd2::birtd_dpo_controller& c);
  void visit(fccd2::birtd_mdpo_controller& c);
  void visit(fccd2::birtd_odpo_controller& c);
  void visit(fccd2::birtd_omdpo_controller& c);

 private:
  /* clang-format off */
  crepr::sim_block3D* m_block;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using block_found_visitor = rpvisitor::filtered_visitor<block_found>;

NS_END(events, d2, cognitive, controller, fordyca);

