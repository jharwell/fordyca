/**
 * \file cache_block_drop.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D1_EVENTS_CACHE_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D1_EVENTS_CACHE_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/ds/operations/cell2D_op.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fordyca.hpp"
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
} // namespace fordyca::controller::cognitive

NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_block_drop
 * \ingroup controller cognitive d1 events
 *
 * \brief Created whenever a robot drops a block in a cache to handle updates
 * needed by robot controllers. Complement to \ref arena_cache_block_drop.
 */
class cache_block_drop : public rer::client<cache_block_drop>,
                         public cdops::cell2D_op {
 private:
  struct visit_typelist_impl {
    using inherited = cdops::cell2D_op::visit_typelist;
    using controllers = controller::d1::typelist;

    using others = rmpl::typelist<fsm::block_to_goal_fsm,
                                  fspds::dpo_semantic_map,
                                  carepr::arena_cache,
                                  tasks::d1::harvester>;

    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<others::type, controllers::type>,
        inherited::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~cache_block_drop(void) override = default;

  cache_block_drop(const cache_block_drop& op) = delete;
  cache_block_drop& operator=(const cache_block_drop& op) = delete;

  /* controllers */
  void visit(controller::cognitive::d1::bitd_dpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_mdpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_odpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_omdpo_controller& controller);

  /* tasks */
  void visit(tasks::d1::harvester& task);

  /* FSMs */
  void visit(cfsm::cell2D_fsm& fsm);
  void visit(fsm::block_to_goal_fsm& fsm);

  /* data structures */
  void visit(class cds::cell2D& cell);
  void visit(fspds::dpo_semantic_map& map);

 protected:
  /**
   * \brief Initialize a cache_block_drop event
   *
   * \param block Block to drop (owned by robot).
   * \param cache Cache to drop into (owned by arena).
   * \param resolution Arena resolution.
   */
  cache_block_drop(std::unique_ptr<crepr::base_block3D> block,
                         carepr::arena_cache* cache,
                         const rtypes::discretize_ratio& resolution);

  carepr::arena_cache* cache(void) const { return m_cache; }
  const crepr::base_block3D* block(void) const { return m_block.get(); }

 private:
  /* clang-format off */
  void dispatch_cache_interactor(tasks::base_foraging_task* task);

  const rtypes::discretize_ratio       mc_resolution;

  std::unique_ptr<crepr::base_block3D> m_block;
  carepr::arena_cache*                 m_cache;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
NS_START(detail);
using cache_block_drop_visitor_impl =
    rpvisitor::precise_visitor<cache_block_drop,
                               cache_block_drop::visit_typelist>;

NS_END(detail);

class cache_block_drop_visitor
    : public detail::cache_block_drop_visitor_impl {
 public:
  using detail::cache_block_drop_visitor_impl:: cache_block_drop_visitor_impl;
};

NS_END(events, d1, cognitive, controller,fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_D1_EVENTS_CACHE_BLOCK_DROP_HPP_ */
