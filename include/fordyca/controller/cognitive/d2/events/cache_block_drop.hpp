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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/controller/cognitive/d1/events/cache_block_drop.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/

NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_block_drop
 * \ingroup controller cognitive d2 events
 *
 * \brief Created whenever a robot drops a block in a cache to handle updates
 * needed by robot controllers. Complement to \ref arena_cache_block_drop.
 */
class cache_block_drop : public rer::client<cache_block_drop>,
                         public fccd1::events::cache_block_drop {
 private:
  struct visit_typelist_impl {
    using controllers = controller::d2::typelist;

    using others = rmpl::typelist<tasks::d2::cache_transferer>;

    using value = boost::mpl::joint_view<others::type, controllers::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~cache_block_drop(void) override = default;

  cache_block_drop(const cache_block_drop& op) = delete;
  cache_block_drop& operator=(const cache_block_drop& op) = delete;

  /* controllers */
  void visit(fccd2::birtd_dpo_controller& controller);
  void visit(fccd2::birtd_mdpo_controller& controller);
  void visit(fccd2::birtd_odpo_controller& controller);
  void visit(fccd2::birtd_omdpo_controller& controller);

  /* tasks */
  void visit(ftasks::d2::cache_transferer& task);

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

 private:
  bool dispatch_cache_interactor(ftasks::base_foraging_task* task,
                                 fccognitive::cache_sel_matrix* csel_matrix);
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

NS_END(events, d2, cognitive, controller, fordyca);

