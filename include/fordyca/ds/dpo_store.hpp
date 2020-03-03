/**
 * \file dpo_store.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/foraging/repr/base_cache.hpp"
#include "cosm/repr/base_block2D.hpp"

#include "fordyca/ds/dp_block_map.hpp"
#include "fordyca/ds/dp_cache_map.hpp"
#include "fordyca/ds/dpo_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);
namespace config { namespace perception {
struct pheromone_config;
}} // namespace config::perception
NS_START(ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_store
 * \ingroup ds
 *
 * \brief Stores sets of caches and blocks the robot encounters in the arena,
 * along with the pheromone-based relevance for each. Also contains the logic
 * for correctly updating the sets of caches and blocks when updated versions of
 * an entity are encountered (e.g. the real version has vanished and the tracked
 * version is out of date).
 */
class dpo_store final : public rer::client<dpo_store> {
 public:
  template <typename T>
  using dpo_entity = repr::dpo_entity<T>;

  enum update_status {
    kNO_CHANGE,
    kNEW_BLOCK_ADDED,
    kBLOCK_MOVED,
    kNEW_CACHE_ADDED,
    kCACHE_UPDATED /* # blocks can change */
  };

  /**
   * \brief The result of a applying a DPO store update.
   *
   * status: Has the store changed as a result of the applied update?
   * reason: If the store has changed, what is the reason for the change?
   * old_loc: If the applied update resulted in a block changing position, then
   *          this field is the previous location of the tracked block.
   */

  struct update_res_t {
    bool status{false};
    update_status reason{kNO_CHANGE};
    rmath::vector2u old_loc{};
  };

  /**
   * \brief The maximum pheromone level if repeat deposits of pheromone are not
   * enabled.
   */
  static constexpr double kNRD_MAX_PHEROMONE = 1.0;

  explicit dpo_store(const config::perception::pheromone_config* config);

  /**
   * \brief Get all blocks the robot is currently aware of, and their
   * corresponding pheromone levels.
   */
  ds::dp_block_map& blocks(void) { return m_blocks; }
  const ds::dp_block_map& blocks(void) const { return m_blocks; }

  /**
   * \brief Get all caches the robot is currently aware of, and their
   * corresponding pheromone levels.
   */
  ds::dp_cache_map& caches(void) { return m_caches; }
  const ds::dp_cache_map& caches(void) const { return m_caches; }

  bool repeat_deposit(void) const { return mc_repeat_deposit; }

  /**
   * \brief Update the densities of all objects in the store (i.e. decay
   * them). Should be called when one unit of time has passed (e.g. every
   * timestep).
   */
  void decay_all(void);

  void clear_all(void);

  bool contains(const crepr::base_block2D* block) const RCSW_PURE;
  bool contains(const cfrepr::base_cache* cache) const;

  const dp_cache_map::value_type* find(const cfrepr::base_cache* cache) const;
  dp_cache_map::value_type* find(const cfrepr::base_cache* cache);

  const dp_block_map::value_type* find(
      const crepr::base_block2D* block) const RCSW_PURE;
  dp_block_map::value_type* find(const crepr::base_block2D* block) RCSW_PURE;

  /**
   * \brief Update the known caches set with the new cache.
   *
   * If there is already a known cache at the location of the incoming cache, it
   * is removed and replaced with a new one.
   *
   * \param cache Cache to update.
   */
  update_res_t cache_update(dpo_entity<cfrepr::base_cache> cache);

  /*
   * \brief Update the known blocks set with the new block.
   *
   * If the block is already in our set of known blocks it *MAY* need to be
   * removed, depending on if its location has changed or not. If its location
   * has not changed, then its pheromone count is updated to correspond to the
   * new block, but the block object currently in the set is not replaced.
   *
   * \return \c TRUE if a block was added, and \c FALSE otherwise.
   */
  update_res_t block_update(dpo_entity<crepr::base_block2D> block_in);

  /**
   * \brief Remove a cache from the set of of known caches.
   */
  bool cache_remove(cfrepr::base_cache* victim);

  /*
   * \brief Remove a block from the set of known blocks. If the victim is not
   * currently in our set of known blocks, no action is performed.
   *
   * \return \c TRUE if a block was removed, \c FALSE otherwise.
   */
  bool block_remove(crepr::base_block2D* victim);

  double pheromone_rho(void) const { return mc_pheromone_rho; }

  boost::optional<rmath::vector2d> last_block_loc(void) const {
    return m_last_block_loc;
  }
  boost::optional<rmath::vector2d> last_cache_loc(void) const {
    return m_last_cache_loc;
  }

 private:
  /* clang-format off */
  const bool                       mc_repeat_deposit;
  const double                     mc_pheromone_rho;

  ds::dp_block_map                 m_blocks{};
  ds::dp_cache_map                 m_caches{};
  boost::optional<rmath::vector2d> m_last_block_loc{};
  boost::optional<rmath::vector2d> m_last_cache_loc{};
  /* clang-format on */
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_DPO_STORE_HPP_ */
