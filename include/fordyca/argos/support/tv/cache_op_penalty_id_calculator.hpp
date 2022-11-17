/**
 * \file cache_op_penalty_id_calculator.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/foraging/tv/penalty_id_calculator.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/argos/support/tv/cache_op_src.hpp"
#include "fordyca/argos/support/tv/op_filter_result.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_op_penalty_id_calculator
 * \ingroup argos support tv
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

NS_END(tv, support, argos, fordyca);

