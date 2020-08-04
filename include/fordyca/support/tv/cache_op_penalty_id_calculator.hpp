/**
 * \file cache_op_penalty_id_calculator.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_CACHE_OP_PENALTY_ID_CALCULATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_CACHE_OP_PENALTY_ID_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/foraging/tv/penalty_id_calculator.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/support/tv/cache_op_src.hpp"
#include "fordyca/support/tv/op_filter_result.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_op_penalty_id_calculator
 * \ingroup support tv
 *
 * \brief Calculates the ID that should be associated with a particular temporal
 * penalty relating to cache operations.
 *
 * \note This class does not really do much now, and is not REALLY necessary,
 * but a similar class IS necessary for calculating block op IDs, and I'm trying
 * to future proof the design here.
 */
class cache_op_penalty_id_calculator : public rer::client<cache_op_penalty_id_calculator>,
                                       public rpdecorator::decorator<cforaging::tv::penalty_id_calculator> {
 public:
  cache_op_penalty_id_calculator(void);

  /* Not copy constructable/assignable by default */
  cache_op_penalty_id_calculator(const cache_op_penalty_id_calculator&) = delete;
  const cache_op_penalty_id_calculator& operator=(const cache_op_penalty_id_calculator&) = delete;

  rtypes::type_uuid operator()(cache_op_src src, op_filter_result filter) const;
};

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_CACHE_OP_PENALTY_ID_CALCULATOR_HPP_ */
