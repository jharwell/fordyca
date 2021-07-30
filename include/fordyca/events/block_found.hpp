/**
 * \file block_found.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_BLOCK_FOUND_HPP_
#define INCLUDE_FORDYCA_EVENTS_BLOCK_FOUND_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/subsystem/perception/model_update_result.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class block_found
 * \ingroup events detail
 *
 * \brief Event that is created whenever a block (possibly known, possibly
 * unknown) appears in a robot's LOS.
 */
class block_found : public rer::client<block_found>, public cdops::cell2D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_op::visit_typelist;
    using controllers = controller::d2::typelist;
    using others = rmpl::typelist<fspds::dpo_store, fspds::dpo_semantic_map>;

    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<inherited::type, controllers::type>::type,
        others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit block_found(crepr::base_block3D* block);
  ~block_found(void) override = default;

  block_found(const block_found&) = delete;
  block_found& operator=(const block_found&) = delete;

  /* DPO foraging */
  fsperception::model_update_result visit(fspds::dpo_store& store);

  /* MDPO foraging */
  void visit(cds::cell2D& cell);
  void visit(cfsm::cell2D_fsm& fsm);
  void visit(fspds::dpo_semantic_map& map);

  /* d2 foraging */
  void visit(controller::cognitive::d2::birtd_dpo_controller& c);
  void visit(controller::cognitive::d2::birtd_mdpo_controller& c);
  void visit(controller::cognitive::d2::birtd_odpo_controller& c);
  void visit(controller::cognitive::d2::birtd_omdpo_controller& c);

 private:
  /* clang-format off */
  crepr::base_block3D* m_block;
  /* clang-format on */
};

NS_END(detail);

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using block_found_visitor = rpvisitor::filtered_visitor<detail::block_found>;

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_FOUND_HPP_ */
