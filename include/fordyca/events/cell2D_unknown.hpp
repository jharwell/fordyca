/**
 * \file cell2D_unknown.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/operations/cell2D_unknown.hpp"

#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell2D_unknown
 * \ingroup events detail
 *
 * \brief Created whenever a cell within an occupancy grid needs to go into an
 * unknown state.
 *
 * This happens in two cases:
 *
 * 1. After its relevance expires.
 * 2. Before the robot sees it for the first time (ala Fog of War).
 */
class cell2D_unknown : public cdops::cell2D_unknown,
                       public rer::client<cell2D_unknown> {
 private:
  struct visit_typelist_impl {
    using inherited = cdops::cell2D_unknown::visit_typelist;
    using others = rmpl::typelist<fspds::occupancy_grid>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  /* parent class visit functions */
  using cdops::cell2D_unknown::visit;

  explicit cell2D_unknown(const rmath::vector2z& coord)
      : cdops::cell2D_unknown(coord),
        ER_CLIENT_INIT("fordyca.events.cell2D_unknown") {}

  void visit(fspds::occupancy_grid& grid);
};

NS_END(detail);

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell2D_unknown_visitor =
    rpvisitor::filtered_visitor<detail::cell2D_unknown>;

NS_END(events, fordyca);
