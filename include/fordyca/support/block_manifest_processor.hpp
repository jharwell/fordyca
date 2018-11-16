/**
 * @file block_manifest_processor.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_MANIFEST_PROCESSOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_MANIFEST_PROCESSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "fordyca/params/arena/block_manifest.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/factory/sharing_factory.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace rmath = rcppsw::math;
namespace representation {
class base_block;
}
NS_START(support);
namespace factory = rcppsw::patterns::factory;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class block_manifest_processor
    : private factory::sharing_factory<representation::base_block,
                                       const rmath::vector2d&,
                                       int> {
 public:
  using block_vector = std::vector<std::shared_ptr<representation::base_block>>;
  explicit block_manifest_processor(const params::arena::block_manifest* const m);

  block_vector create_blocks(void);

 private:
  // clang-format off
  const params::arena::block_manifest mc_manifest;
  // clang-format on
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_MANIFEST_PROCESSOR_HPP_ */
