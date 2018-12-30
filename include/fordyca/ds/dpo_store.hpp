/**
 * @file dpo_store.hpp
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

#ifndef INCLUDE_FORDYCA_DS_DPO_STORE_HPP_
#define INCLUDE_FORDYCA_DS_DPO_STORE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/ds/dpo_set.hpp"
#include "fordyca/ds/dp_block_set.hpp"
#include "fordyca/ds/dp_cache_set.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);
namespace params { namespace perception { struct pheromone_params; }}
NS_START(ds);

namespace visitor = rcppsw::patterns::visitor;
namespace er = rcppsw::er;
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class dpo_store
 * @ingroup ds
 *
 * @brief Stores sets of caches and blocks the robot encounters in the arena,
 * along with the pheromone-based relevance for each. Also contains the logic
 * for correctly updating the sets of caches and blocks when updated versions of
 * an entity are encountered (e.g. the real version has vanished and the tracked
 * version is out of date).
 */
class dpo_store : public er::client<dpo_store>,
                  public visitor::visitable_any<dpo_store> {
 public:
  template<typename T>
  using dp_entity = representation::dp_entity<T>;

  enum update_status {
    kNoChange,
    kNewBlockAdded,
    kBlockMoved,
    kNewCacheAdded,
    kCacheUpdated /* # blocks can change */
  };

  /**
   * @brief The result of a applying a DPO store update.
   *
   * status: Has the store changed as a result of the applied update?
   * reason: If the store has changed, what is the reason for the change?
   * old_loc: If the applied update resulted in a block changing position, then
   *          this field is the previous location of the tracked block.
   */

  struct update_res_t {
    bool status;
    update_status reason;
    rmath::vector2u old_loc;
  };

  /**
   * @brief The maximum pheromone level if repeat deposits of pheromone are not
   * enabled.
   */
  static constexpr double kNRD_MAX_PHEROMONE = 1.0;

  explicit dpo_store(const params::perception::pheromone_params* params);

  /**
   * @brief Get all blocks the robot is currently aware of, and their
   * corresponding pheromone levels.
   */
  ds::dp_block_set& blocks(void) { return m_blocks; }
  const ds::dp_block_set& blocks(void) const { return m_blocks; }

  /**
   * @brief Get all caches the robot is currently aware of, and their
   * corresponding pheromone levels.
   */
  ds::dp_cache_set& caches(void) { return m_caches; }
  const ds::dp_cache_set& caches(void) const { return m_caches; }

  bool repeat_deposit(void) const { return mc_repeat_deposit; }
  double rho(void) const { return mc_rho; }

  /**
   * @brief Update the densities of all objects in the store (i.e. decay
   * them). Should be called when one unit of time has passed (e.g. every
   * timestep).
   */
  void decay_all(void);

  void clear_all(void);

  bool contains(const std::shared_ptr<representation::base_block>& block) const;
  bool contains(const std::shared_ptr<representation::base_cache>& cache) const;

  const const_dp_block_set::value_type* find(
      const std::shared_ptr<representation::base_block>& block) const;

  const const_dp_cache_set::value_type* find(
      const std::shared_ptr<representation::base_cache>& cache) const;

  /**
   * @brief Update the known caches set with the new cache.
   *
   * If there is already a known cache at the location of the incoming cache, it
   * is removed and replaced with a new one.
   *
   * @param cache Cache to add.
   */
  update_res_t cache_update(const dp_entity<representation::base_cache>& cache);

  /*
   * @brief Update the known blocks set with the new block.
   *
   * If the block is already in our set of known blocks it *MAY* need to be
   * removed, depending on if its location has changed or not. If its location
   * has not changed, then its pheromone count is updated to correspond to the
   * new block, but the block object currently in the set is not replaced.
   *
   * @return \c TRUE if a block was added, and \c FALSE otherwise.
   */
  update_res_t block_update(const dp_entity<representation::base_block>& block);

  /**
   * @brief Remove a cache from the set of of known caches.
   */
  bool cache_remove(const std::shared_ptr<representation::base_cache>& victim);

  /*
   * @brief Remove a block from the set of known blocks. If the victim is not
   * currently in our set of known blocks, no action is performed.
   *
   * @return \c TRUE if a block was removed, \c FALSE otherwise.
   */
  bool block_remove(const std::shared_ptr<representation::base_block>& victim);

 private:
  /*
   * Sets are used for object storage because there is no concept of order
   * among the known blocks/caches.
   */
  /* clang-format off */
  bool             mc_repeat_deposit;
  double           mc_rho;
  ds::dp_block_set m_blocks{};
  ds::dp_cache_set m_caches{};
  /* clang-format on */
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_DPO_STORE_HPP_ */
