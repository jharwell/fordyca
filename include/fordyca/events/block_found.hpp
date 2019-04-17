/**
 * @file block_found.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace repr {
class base_block;
}
namespace controller { namespace depth2 {
class grp_dpo_controller;
class grp_mdpo_controller;
}} // namespace controller::depth2

namespace ds {
class dpo_semantic_map;
class dpo_store;
} // namespace ds

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct block_found_visit_set {
  using inherited = cell_op_visit_set::value;

  using defined =
      rvisitor::precise_visit_set<controller::depth2::grp_dpo_controller,
                                 controller::depth2::grp_mdpo_controller,
                                 ds::dpo_store,
                                 ds::dpo_semantic_map>;

  using value = boost::mpl::joint_view<inherited::type, defined::type>;
};

/**
 * @class block_found
 * @ingroup events detail
 *
 * @brief Event that is created whenever a block (possibly known, possibly
 * unknown) appears in a robot's LOS.
 */
class block_found : public rcppsw::er::client<block_found>, public cell_op {
 public:
  explicit block_found(std::unique_ptr<repr::base_block> block);
  explicit block_found(const std::shared_ptr<repr::base_block>& block);
  ~block_found(void) override = default;

  block_found(const block_found& op) = delete;
  block_found& operator=(const block_found& op) = delete;

  /* DPO foraging */
  void visit(ds::dpo_store& store);

  /* MDPO foraging */
  void visit(ds::cell2D& cell);
  void visit(fsm::cell2D_fsm& fsm);
  void visit(ds::dpo_semantic_map& map);

  /* depth2 foraging */
  void visit(controller::depth2::grp_dpo_controller& c);
  void visit(controller::depth2::grp_mdpo_controller& c);

 private:
  void pheromone_update(ds::dpo_semantic_map& map);

  /* clang-format off */
  std::shared_ptr<repr::base_block> m_block;
  /* clang-format on */
};

/**
 * @brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using block_found_visitor_impl =
    rvisitor::precise_visitor<detail::block_found,
                             detail::block_found_visit_set::value>;

NS_END(detail);

class block_found_visitor : public detail::block_found_visitor_impl {
  using detail::block_found_visitor_impl::block_found_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_FOUND_HPP_ */
