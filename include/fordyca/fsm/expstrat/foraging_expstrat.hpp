/**
 * \file foraging_expstrat.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_EXPSTRAT_FORAGING_EXPSTRAT_HPP_
#define INCLUDE_FORDYCA_FSM_EXPSTRAT_FORAGING_EXPSTRAT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/fsm/expstrat/base_expstrat.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/subsystem_fwd.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class cache_sel_matrix;
class block_sel_matrix;
} /* namespace controller */

namespace ds {
class dpo_store;
} /* namespace ds */

NS_START(fsm, expstrat);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_expstrat
 * \ingroup fsm expstrat
 *
 * \brief Base class for different exploration behaviors that controller can
 * exhibit when looking for stuff.
 */
class foraging_expstrat : public cfsm::expstrat::base_expstrat,
                          public rpprototype::clonable<foraging_expstrat> {
 public:
  foraging_expstrat(crfootbot::footbot_saa_subsystem* saa,
                    rmath::rng* rng) :
      base_expstrat(saa),
      m_rng(rng) {}

  struct params {
    params(crfootbot::footbot_saa_subsystem* const saa_in,
           const controller::block_sel_matrix *const bsel_matrix_in,
           const controller::cache_sel_matrix *const csel_matrix_in,
           const ds::dpo_store *const dpo_store_in,
           const rutils::color& ledtaxis_target_in = rutils::color())
        : saa(saa_in),
          bsel_matrix(bsel_matrix_in),
          csel_matrix(csel_matrix_in),
        dpo_store(dpo_store_in),
        ledtaxis_target(ledtaxis_target_in) {}

    crfootbot::footbot_saa_subsystem* saa;
    const controller::block_sel_matrix *bsel_matrix;
    const controller::cache_sel_matrix *csel_matrix;
    const ds::dpo_store *dpo_store;
    rutils::color ledtaxis_target;
  };
  foraging_expstrat(const foraging_expstrat&) = delete;
  foraging_expstrat& operator=(const foraging_expstrat&) = delete;

 protected:
  crfootbot::footbot_saa_subsystem* saa(void) const;
  crfootbot::footbot_saa_subsystem* saa(void);
  rmath::rng* rng(void) { return m_rng; }
  rmath::rng* rng(void) const { return m_rng; }

 private:
  /* clang-format off */
  rmath::rng* m_rng;
  /* clang-format on */
};

NS_END(expstrat, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPSTRAT_FORAGING_EXPSTRAT_HPP_ */
