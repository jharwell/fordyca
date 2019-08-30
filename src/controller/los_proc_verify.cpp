/**
 * @file los_proc_verify.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/los_proc_verify.hpp"

#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/repr/base_cache.hpp"
#include "fordyca/repr/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool los_proc_verify::operator()(const ds::dpo_store* const c_dpo) const {
  /*
   * Verify that for each cell that contained a block in the LOS, that the block
   * is also contained in the store after processing. If we are using oracular
   * controllers, then it is possible to be told about the existence of a block
   * that is hidden by a cache, which will be removed during normal LOS
   * processing from our DPO store, but will trigger an unconditional assert()
   * here. The fix is to only assert() if there is not a cache that contains the
   * block's location, and it is therefore not occluded.
   */
  for (auto& cache : c_dpo->caches().const_values_range()) {
    for (auto& block : mc_los->blocks()) {
      if (!cache.ent()->contains_point(block->rloc())) {
        ER_ASSERT(c_dpo->contains(block),
                  "Store does not contain block%d@%s",
                  block->id(),
                  block->dloc().to_str().c_str());
      }
    } /* for(&block..) */
  }   /* for(&cache..) */

  /*
   * Verify that for each cell that contained a cache in the LOS:
   *
   * - The corresponding cache exists in the DPO store.
   * - The DPO store and LOS versions of the cache have the same # of blocks,
   *   location, and ID.
   */
  for (auto& c1 : mc_los->caches()) {
    auto exists = c_dpo->find(c1);
    ER_ASSERT(nullptr != exists,
              "LOS Cache%d@%s does not exist in DPO store",
              c1->id(),
              c1->dloc().to_str().c_str());
    ER_ASSERT(c1->dloc() == exists->ent()->dloc(),
              "LOS/DPO store disagree on cache%d location: %s/%s",
              c1->id(),
              c1->dloc().to_str().c_str(),
              exists->ent()->dloc().to_str().c_str());
    ER_ASSERT(c1->id() == exists->ent()->id(),
              "DPO store/LOS disagree on cache ID @%s: %d/%d",
              c1->dloc().to_str().c_str(),
              exists->ent()->id(),
              c1->id());
    ER_ASSERT(c1->n_blocks() == exists->ent()->n_blocks(),
              "LOS/DPO store disagree on # of blocks in cache%d@%s: %zu/%zu",
              c1->id(),
              c1->dloc().to_str().c_str(),
              c1->n_blocks(),
              exists->ent()->n_blocks());
  } /* for(c1..) */
  return true;
} /* operator()() */

bool los_proc_verify::operator()(const ds::dpo_semantic_map* const c_map) const {
  ER_ASSERT(this->operator()(c_map->store()),
            "DPO LOS processing verification failed");
  /*
   * Verify that for each cell that contained a block in the LOS, the
   * corresponding cell in the map also contains the same block.
   */
  for (auto& block : mc_los->blocks()) {
    auto& cell = c_map->access<ds::occupancy_grid::kCell>(block->dloc());
    ER_ASSERT(cell.state_has_block(),
              "Cell@%s not in HAS_BLOCK state",
              block->dloc().to_str().c_str());
    ER_ASSERT(cell.block()->id() == block->id(),
              "Cell@%s has wrong block ID (%u vs %u)",
              block->dloc().to_str().c_str(),
              block->id(),
              cell.block()->id());
  } /* for(&block..) */

  /*
   * Verify that for each cell in LOS that was empty or contained a block, that
   * it matches the map version.
   */
  for (uint i = 0; i < mc_los->xsize(); ++i) {
    for (uint j = 0; j < mc_los->ysize(); ++j) {
      rmath::vector2u d = mc_los->cell(i, j).loc();
      auto& cell1 = mc_los->cell(i, j);
      auto& cell2 = c_map->access<ds::occupancy_grid::kCell>(d);

      if (cell1.state_has_block() || cell1.state_is_empty()) {
        ER_ASSERT(cell1.fsm().current_state() == cell2.fsm().current_state(),
                  "LOS/DPO map disagree on state of cell@%s: %d/%d",
                  d.to_str().c_str(),
                  cell1.fsm().current_state(),
                  cell2.fsm().current_state());
        if (cell1.state_has_block()) {
          ER_ASSERT(cell1.block()->id() == cell2.block()->id(),
                    "LOS/DPO map disagree on block id in cell@%s: %d/%d",
                    d.to_str().c_str(),
                    cell1.block()->id(),
                    cell2.block()->id());
        }
      }
    } /* for(j..) */
  }   /* for(i..) */
  return true;
} /* operator()() */

NS_END(controller, fordyca);
