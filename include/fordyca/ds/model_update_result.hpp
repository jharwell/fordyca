/**
 * \file model_update_result.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_DS_MODEL_UPDATE_RESULT_HPP_
#define INCLUDE_FORDYCA_DS_MODEL_UPDATE_RESULT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "fordyca/ds/model_update_status.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
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

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_MODEL_UPDATE_RESULT_HPP_ */
