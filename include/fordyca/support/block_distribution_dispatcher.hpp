/**
 * @file block_distribution_dispatcher.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_DISTRIBUTION_DISPATCHER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_DISTRIBUTION_DISPATCHER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>

#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "fordyca/params/block_distribution_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class block;
class arena_grid;
class multicell_entity;
} // namespace representation

NS_START(support);
class base_block_distributor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_distribution_dispatcher
 * @ingroup support
 *
 * @brief Distributes all blocks as directed on simulation start, and then
 * re-dstributes individual blocks every time they are dropped in the nest.
 */
class block_distribution_dispatcher : public rcppsw::er::client {
 public:
  static constexpr char kDIST_SINGLE_SRC[] = "single_source";
  static constexpr char kDIST_RANDOM[] = "random";
  static constexpr char kDIST_POWERLAW[] = "powerlaw";

  using entity_list = std::list<const representation::multicell_entity*>;
  using block_vector = std::vector<std::shared_ptr<representation::block>>;

  block_distribution_dispatcher(std::shared_ptr<rcppsw::er::server> server,
                                representation::arena_grid& grid,
                                const struct params::block_distribution_params* params);
  ~block_distribution_dispatcher(void);

  block_distribution_dispatcher(const block_distribution_dispatcher& s) = delete;
  block_distribution_dispatcher& operator=(const block_distribution_dispatcher& s) = delete;

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
  bool distribute_block(std::shared_ptr<representation::block>& block,
                        entity_list& entities);

  /**
   * @brief Distribute all blocks in the arena.
   *
   * @return \c TRUE iff distribution was successful, \c FALSE otherwise.
   */
  bool distribute_blocks(block_vector& blocks, entity_list& entities);

 private:
  // clang-format off
  std::string                                    m_dist_type;
  const struct params::block_distribution_params mc_params;
  representation::arena_grid&                    m_grid;
  std::unique_ptr<base_block_distributor>        m_dist;
  // clang-format on
};

NS_END(support, fordyca);
#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_DISTRIBUTION_DISPATCHER_HPP_ */
