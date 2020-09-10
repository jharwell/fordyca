/**
 * \file acquire_existing_cache_fsm.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_ACQUIRE_EXISTING_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_ACQUIRE_EXISTING_CACHE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <memory>
#include <utility>

#include "rcppsw/math/rng.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/spatial/fsm/acquire_goal_fsm.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/fsm_ro_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::robots::footbot {
class footbot_saa_subsystem;
} /* namespace cosm::robots::footbot */

NS_START(fordyca);

namespace controller::cognitive {
class cache_sel_matrix;
}
namespace repr {
class cache;
}
namespace ds {
class dpo_store;
}

NS_START(fsm);

namespace expstrat {
class foraging_expstrat;
} /* namespace expstrat */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class acquire_existing_cache_fsm
 * \ingroup fsm d1
 *
 * \brief Acquire an existing cache within the arena. Once such a cache has been
 * acquired (always by vectoring), it signals that it has completed its task.
 */
class acquire_existing_cache_fsm
    : public rer::client<acquire_existing_cache_fsm>,
      public csfsm::acquire_goal_fsm {
 public:
  /**
   * \param for_pickup Are we acquiring a cache for pickup or block drop?
   */
  acquire_existing_cache_fsm(
      const fsm_ro_params* c_params,
      crfootbot::footbot_saa_subsystem* saa,
      std::unique_ptr<csexpstrat::base_expstrat> exp_behavior,
      rmath::rng* rng,
      bool for_pickup);

  ~acquire_existing_cache_fsm(void) override = default;

  acquire_existing_cache_fsm(const acquire_existing_cache_fsm&) = delete;
  acquire_existing_cache_fsm& operator=(const acquire_existing_cache_fsm&) =
      delete;

 private:
  using acq_loc_type = std::pair<rtypes::type_uuid, rmath::vector2d>;

  /*
   * See \ref acquire_goal_fsm for the purpose of these callbacks.
   */
  static csmetrics::goal_acq_metrics::goal_type acq_goal_internal(void)
      RCSW_CONST;

  boost::optional<acquire_goal_fsm::candidate_type> existing_cache_select(void);
  bool candidates_exist(void) const RCSW_PURE;
  boost::optional<acq_loc_type> calc_acq_location(void);
  bool cache_acq_valid(const rmath::vector2d& loc, const rtypes::type_uuid& id);

  bool cache_acquired_cb(bool explore_result) const;

  /* clang-format off */
  /**
   * \brief Needed to ensure the points that robots choose when acquiring the
   * cache are far enough inside the cache extent that all ground sensors are
   * guaranteed to read "in cache". Set to footbot radius + a little padding.
   */
  static constexpr double kFOOTBOT_CACHE_ACQ_FACTOR = 0.2;

  const bool                                           mc_for_pickup;
  const controller::cognitive::cache_sel_matrix* const mc_matrix;
  const ds::dpo_store*                const            mc_store;
  /* clang-format on */
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_ACQUIRE_EXISTING_CACHE_FSM_HPP_ */
