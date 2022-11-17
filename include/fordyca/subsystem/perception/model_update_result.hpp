/**
 * \file model_update_result.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "fordyca/subsystem/perception/model_update_status.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct model_update_resultg
 * \ingroup subsystem perception
 *
 * \brief The result of a applying an update to a \ref base_perception_model
 * derived class.
 */
struct model_update_result {
  /**
   * If the model has changed, what is the reason for the change?
   */
  model_update_status reason{ model_update_status::ekNO_CHANGE };

  /**
   * If the applied update resulted in an object changing position, then this
   * field is the previous location of the tracked object.
   */
  rmath::vector2z old_loc{};
};

NS_END(perception, subsystem, fordyca);

