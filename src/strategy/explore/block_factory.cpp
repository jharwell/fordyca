/**
 * \file block_factory.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/strategy/explore/block_factory.hpp"

#include "fordyca/strategy/explore/crw_adaptor.hpp"
#include "fordyca/strategy/explore/likelihood_block_search.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_factory::block_factory(void) {
  register_type<crw_adaptor>(kCRW);
  register_type<likelihood_block_search>(kLikelihoodSearch);
}

NS_END(explore, strategy, fordyca);
