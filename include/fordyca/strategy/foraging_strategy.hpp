/**
 * \file foraging_strategy.hpp
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

#ifndef INCLUDE_FORDYCA_STRATEGY_FORAGING_STRATEGY_HPP_
#define INCLUDE_FORDYCA_STRATEGY_FORAGING_STRATEGY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/base_strategy.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"
#include "fordyca/subsystem/perception/known_objects_accessor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller::cognitive {
class cache_sel_matrix;
class block_sel_matrix;
} // namespace controller::cognitive

NS_START(strategy);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_strategy
 * \ingroup strategy
 *
 * \brief Base class for different behaviors that controllers can
 * exhibit when looking for stuff, avoiding collision, etc.
 */
class foraging_strategy : public csstrategy::base_strategy {
 public:
  struct params {
    params(csubsystem::saa_subsystemQ3D* const saa_in,
           const controller::cognitive::block_sel_matrix* const bsel_matrix_in,
           const controller::cognitive::cache_sel_matrix* const csel_matrix_in,
           const fsperception::known_objects_accessor* accesor_in,
           const rutils::color& ledtaxis_target_in)
        : saa(saa_in),
          bsel_matrix(bsel_matrix_in),
          csel_matrix(csel_matrix_in),
          accessor(accesor_in),
          ledtaxis_target(ledtaxis_target_in) {}

    csubsystem::saa_subsystemQ3D* saa;
    const controller::cognitive::block_sel_matrix* bsel_matrix;
    const controller::cognitive::cache_sel_matrix* csel_matrix;
    const fsperception::known_objects_accessor* accessor;
    rutils::color ledtaxis_target;
  };

  foraging_strategy(csubsystem::saa_subsystemQ3D* saa,
                    const fsperception::known_objects_accessor* accessor,
                    rmath::rng* rng)
      : base_strategy(saa, rng),
        mc_accessor(accessor) {}

  foraging_strategy(const foraging_strategy&) = delete;
  foraging_strategy& operator=(const foraging_strategy&) = delete;

 protected:
  const fsperception::known_objects_accessor* accessor(void) const {
    return mc_accessor;
  }

  /* clang-format off */
  const fsperception::known_objects_accessor* mc_accessor;
  /* clang-format on */
};

NS_END(strategy, fordyca);

#endif /* INCLUDE_FORDYCA_STRATEGY_FORAGING_STRATEGY_HPP_ */
