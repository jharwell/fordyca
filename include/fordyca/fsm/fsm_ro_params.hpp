/**
 * \file fsm_ro_params.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/common/common.hpp"
#include "rcppsw/math/vector2.hpp"

#include "fordyca/strategy/config/strategy_config.hpp"
#include "fordyca/subsystem/perception/known_objects_accessor.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

namespace controller::cognitive {
class block_sel_matrix;
class cache_sel_matrix;
} // namespace controller::cognitive

NS_START(fsm);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct fsm_ro_params
 * \ingroup fsm
 *
 * \brief Contains all parameters for FSM initialization that will be read-only
 * by the FSM at run-time; not all FSMs need all members.
 */
struct fsm_ro_params {
  /* clang-format off */
  const fccognitive::block_sel_matrix*        bsel_matrix;
  const fccognitive::cache_sel_matrix*        csel_matrix;
  const fspds::dpo_store*                     store;
  const fsperception::known_objects_accessor* accessor;
  const fsconfig::strategy_config             strategy{};
  /* clang-format on */
};

NS_END(fsm, fordyca);
