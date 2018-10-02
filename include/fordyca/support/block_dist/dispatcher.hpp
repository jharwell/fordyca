/**
 * @file dispatcher.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_DISPATCHER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_DISPATCHER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "fordyca/params/arena/block_dist_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class base_block;
class multicell_entity;
} // namespace representation

namespace ds {
class arena_grid;
} // namespace ds

NS_START(support, block_dist);
class base_distributor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class dispatcher
 * @ingroup block_dist support
 *
 * @brief Dispatches call to distribute blocks (or a single block), as
 * configured in simulation input file.
 *
 * - Single and dual source distribution assumes left-right rectangular arena.
 * - Power law, quad source, random distribution assume square arena.
 */
class dispatcher {
 public:
  static constexpr char kDIST_SINGLE_SRC[] = "single_source";
  static constexpr char kDIST_RANDOM[] = "random";
  static constexpr char kDIST_DUAL_SRC[] = "dual_source";
  static constexpr char kDIST_QUAD_SRC[] = "quad_source";
  static constexpr char kDIST_POWERLAW[] = "powerlaw";

  using entity_list = std::list<const representation::multicell_entity*>;
  using block_vector = std::vector<std::shared_ptr<representation::base_block>>;

  dispatcher(ds::arena_grid& grid,
             const struct params::arena::block_dist_params* params);
  ~dispatcher(void);

  dispatcher(const dispatcher& s) = delete;
  dispatcher& operator=(const dispatcher& s) = delete;

  /**
   * @brief Initialize the selected block distributor. This is a separate
   * function, rather than happening in the constructor, so that error handling
   * can be done without exceptions.
   *
   * @return \c TRUE if initialization successful, \c FALSE otherwise.
   */
  bool initialize(void);

  /**
   * @brief Distribute a block in the arena.
   *
   * @param block The block to distribute.
   * @param entities List of all arena entities in the arena that distribution
   * should treat as obstacles/things that blocks should not be placed in.
   *
   * @return \c TRUE iff distribution was successful, \c FALSE otherwise.
   */
  bool distribute_block(std::shared_ptr<representation::base_block>& block,
                        entity_list& entities);

  /**
   * @brief Distribute all blocks in the arena.
   *
   * @return \c TRUE iff distribution was successful, \c FALSE otherwise.
   */
  bool distribute_blocks(block_vector& blocks, entity_list& entities);

 private:
  // clang-format off
  const struct params::arena::block_dist_params mc_params;
  std::string                                   m_dist_type;
  ds::arena_grid&                               m_grid;
  std::unique_ptr<base_distributor>             m_dist;

  // clang-format on
};

NS_END(block_dist, support, fordyca);
#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_DISPATCHER_HPP_ */
