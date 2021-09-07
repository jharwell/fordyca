/**
 * \file foraging_memory_model.hpp
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

#ifndef INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_FORAGING_MEMORY_MODEL_HPP_
#define INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_FORAGING_MEMORY_MODEL_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/subsystem/perception/base_memory_model.hpp"
#include "cosm/arena/ds/cache_vector.hpp"

#include "fordyca/subsystem/perception/model_update_result.hpp"
#include "fordyca/subsystem/perception/known_objects_accessor.hpp"
#include "fordyca/subsystem/perception/tracked_objects_accessor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_memory_model
 * \ingroup subsystem perception
 *
 * \brief Base class defining the interface for all perception memory models in
 * FORDYCA.
 */
template <typename TTrackedBlockContainer, typename TTrackedCacheContainer>
class foraging_memory_model : public csperception::base_memory_model,
                                  public known_objects_accessor,
                                  public tracked_objects_accessor<TTrackedBlockContainer,
                                                                  TTrackedCacheContainer>,
                                  public rer::client<foraging_memory_model<TTrackedBlockContainer,
                                                                               TTrackedCacheContainer>> {
 public:
  using tracked_objects_accessor_type = tracked_objects_accessor<TTrackedBlockContainer,
                                                        TTrackedCacheContainer>;
  using typename tracked_objects_accessor_type::block_container_type;
  using typename tracked_objects_accessor_type::cache_container_type;
  using typename tracked_objects_accessor_type::tracked_cache_type;
  using typename tracked_objects_accessor_type::tracked_block_type;

  const known_objects_accessor* known_objects(void) const { return this; }
  const tracked_objects_accessor_type* tracked_objects(void) const { return this; }

  foraging_memory_model(void)
      : ER_CLIENT_INIT("fordyca.ds.foraging_memory_model") {}

  /* known_objects_accesor overrides */
  const block_container_type& tracked_blocks(void) const override {
    return m_blocks;
  }
  block_container_type& tracked_blocks(void) override { return m_blocks; }
  const cache_container_type& tracked_caches(void) const override  {
    return m_caches;
  }
  cache_container_type& tracked_caches(void) override { return m_caches; }

  /**
   * \brief Does the model currently contain the specified block (i.e., is it
   * tracked?). Searches by block UUID.
   */
  bool contains(const crepr::base_block3D* block) const {
    return nullptr != find(block);
  }

  /**
   * \brief Does the model currently contain the specified block (i.e., is it
   * tracked?). Searches by cache location (which is always the same).
   */
  bool contains(const carepr::base_cache* cache) const {
    return nullptr != find(cache);
  }

  const tracked_cache_type* find(const carepr::base_cache* cache) const {
    return tracked_caches().find(cache->dcenter2D());
  }
  tracked_cache_type* find(const carepr::base_cache* cache) {
    return tracked_caches().find(cache->dcenter2D());
  }

  const tracked_block_type* find(const crepr::base_block3D* block) const  {
    return tracked_blocks().find(block->id());
  }

  tracked_block_type* find(const crepr::base_block3D* block) {
    return tracked_blocks().find(block->id());
  }

  /**
   * \brief Update the known caches set with the new cache.
   *
   * If there is already a known cache at the location of the incoming cache, it
   * is removed and replaced with a new one.
   *
   * \param cache Cache to update.
   */
  virtual model_update_result cache_update(tracked_cache_type&& cache) = 0;

  /**
   * \brief Update the known blocks set with the new block.
   *
   * If the block is already in our set of known blocks it *MAY* need to be
   * removed, depending on if its location and/or other tracked metadata has
   * changed or not.
   *
   * \return \c TRUE if a block was added, and \c FALSE otherwise.
   */
  virtual model_update_result block_update(tracked_block_type&& block) = 0;

  /**
   * \brief Remove a cache from the set of of known caches.
   */
  virtual bool cache_remove(carepr::base_cache* victim) = 0;

  /*
   * \brief Remove a block from the set of known blocks. If the victim is not
   * currently in our set of known blocks, no action is performed.
   *
   * \return \c TRUE if a block was removed, \c FALSE otherwise.
   */
  virtual bool block_remove(crepr::base_block3D* victim) = 0;

 private:
  /* clang-format off */
  TTrackedBlockContainer m_blocks{};
  TTrackedCacheContainer m_caches{};
  /* clang-format on */
};

NS_END(perception, subsystem, fordyca);

#endif /* INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_FORAGING_MEMORY_MODEL_HPP_ */
