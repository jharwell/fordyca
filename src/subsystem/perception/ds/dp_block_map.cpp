/**
 * \file dp_block_map.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/subsystem/perception/ds/dp_block_map.hpp"

#include <numeric>

#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, ds);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string dp_block_map::to_str(void) const {
  auto range = values_range();
  return std::accumulate(range.begin(),
                         range.end(),
                         std::string(),
                         [&](const std::string& a, const auto& b) {
                           return a + "b" + rcppsw::to_string(b.ent()->id()) +
                                  ",";
                         });
} /* to_str() */

NS_END(ds, perception, subsystem, fordyca);
