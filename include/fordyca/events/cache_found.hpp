/**
 * @file cache_found.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CACHE_FOUND_HPP_
#define INCLUDE_FORDYCA_EVENTS_CACHE_FOUND_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace repr {
class base_cache;
}

namespace ds {
class dpo_store;
class dpo_semantic_map;
} // namespace ds

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/*
 * @class cache_found
 * @ingroup fordyca events detail
 *
 * @brief Created whenever a NEW cache (i.e. one that is not currently known to
 * a robot, but possibly one that it has seen before and whose relevance had
 * expired) is discovered by the robot via it appearing in the robot's LOS.
 */
class cache_found : public cell_op, public rer::client<cache_found> {
 private:
  struct visit_typelist_impl {
    using inherited = cell_op::visit_typelist;
    using others = rmpl::typelist<ds::dpo_store, ds::dpo_semantic_map>;
    using controllers = controller::depth2::typelist;
    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<controllers::type, others::type>,
        inherited::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit cache_found(std::unique_ptr<repr::base_cache> cache);
  explicit cache_found(const std::shared_ptr<repr::base_cache>& cache);
  ~cache_found(void) override = default;

  cache_found(const cache_found& op) = delete;
  cache_found& operator=(const cache_found& op) = delete;

  /* DPO foraging */
  void visit(ds::dpo_store& store);

  /* MDPO foraging */
  void visit(ds::cell2D& cell);
  void visit(ds::dpo_semantic_map& map);
  void visit(fsm::cell2D_fsm& fsm);

  /* depth2 foraging */
  void visit(controller::depth2::grp_dpo_controller& controller);
  void visit(controller::depth2::grp_mdpo_controller& c);
  void visit(controller::depth2::grp_odpo_controller& controller);
  void visit(controller::depth2::grp_omdpo_controller& c);

 private:
  std::shared_ptr<repr::base_cache> m_cache;
};

/**
 * @brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_found_visitor_impl =
    rvisitor::precise_visitor<detail::cache_found,
                              detail::cache_found::visit_typelist>;

NS_END(detail);

class cache_found_visitor : public detail::cache_found_visitor_impl {
  using detail::cache_found_visitor_impl::cache_found_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_FOUND_HPP_ */
