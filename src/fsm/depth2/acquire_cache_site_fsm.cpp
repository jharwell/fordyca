/**
 * @file acquire_cache_site_fsm.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/depth2/acquire_cache_site_fsm.hpp"

#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/controller/depth2/cache_site_selector.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/repr/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth2);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_cache_site_fsm::acquire_cache_site_fsm(
    const controller::cache_sel_matrix* matrix,
    controller::saa_subsystem* const saa,
    ds::dpo_store* const store)
    : ER_CLIENT_INIT("fordyca.fsm.depth2.acquire_cache_site"),
      acquire_goal_fsm(
          saa,
          acquire_goal_fsm::hook_list{
            .acquisition_goal = std::bind(&acquire_cache_site_fsm::acquisition_goal_internal,
                                          this),
                .goal_select = std::bind(&acquire_cache_site_fsm::site_select, this),
                .candidates_exist = std::bind(&acquire_cache_site_fsm::candidates_exist,
                                              this),
                .goal_acquired_cb = std::bind(&acquire_cache_site_fsm::site_acquired_cb,
                                              this,
                                              std::placeholders::_1),
                .explore_term_cb = std::bind(&acquire_cache_site_fsm::site_exploration_term_cb,
                                             this),
                .goal_valid_cb = [](const rmath::vector2d&, uint) noexcept { return true;
}
}),
      mc_matrix(matrix),
      mc_store(store) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_const bool acquire_cache_site_fsm::site_acquired_cb(
    bool explore_result) const {
  ER_ASSERT(!explore_result, "Found cache site by exploring?");
  rmath::vector2d position = saa_subsystem()->sensing()->position();
  for (auto& b : mc_store->blocks().const_values_range()) {
    if ((position - b.ent()->real_loc()).length() <=
        boost::get<double>(mc_matrix->find("block_prox_dist")->second)) {
      ER_WARN("Cannot acquire cache site@%s: Block%d@%s too close",
              position.to_str().c_str(),
              b.ent()->id(),
              b.ent()->real_loc().to_str().c_str());
      return false;
    }
  } /* for(&b..) */

  return true;
} /* site_acquired_cb() */

__rcsw_const bool acquire_cache_site_fsm::site_exploration_term_cb(void) const {
  ER_FATAL_SENTINEL("Cache site acquired through exploration");
  return false;
} /* site_exploration_term_cb() */

boost::optional<acquire_goal_fsm::candidate_type> acquire_cache_site_fsm::site_select(
    void) const {
  if (auto best = controller::depth2::cache_site_selector(
          mc_matrix)(mc_store->caches(),
                     mc_store->blocks(),
                     saa_subsystem()->sensing()->position())) {
    ER_INFO("Select cache site@%s for acquisition", best->to_str().c_str());
    return boost::make_optional(acquire_goal_fsm::candidate_type(
        *best, vector_fsm::kCACHE_SITE_ARRIVAL_TOL, -1));
  } else {
    ER_WARN("No cache site selected for acquisition--internal error?")
    return boost::optional<acquire_goal_fsm::candidate_type>();
  }
} /* site_select() */

__rcsw_const acquisition_goal_type
acquire_cache_site_fsm::acquisition_goal_internal(void) const {
  return acquisition_goal_type::ekCACHE_SITE;
} /* acquisition_goal_internal() */

NS_END(depth2, controller, fordyca);
