
/**
 * \file ntimestep_perception_subsystem.hpp
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

#ifndef INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_NTIMESTEP_PERCEPTION_SUBSYSTEM_HPP_
#define INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_NTIMESTEP_PERCEPTION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/er/client.hpp"

#include "cosm/ds/entity_vector.hpp"

#include "fordyca/subsystem/perception/foraging_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/foraging_perception_model.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca::ds {
class nb_store;
}

NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ntimestep_perception_subsystem
 * \ingroup controller cognitive
 *
 * \brief Translates the sensor readings of the robot (i.e. \ref forager_los),
 * into a useful internal repr: a \ref dpo_store.
 */
class ntimestep_perception_subsystem final
    : public rer::client<ntimestep_perception_subsystem>,
      public foraging_perception_subsystem {

 public:
  explicit ntimestep_perception_subsystem(const cspconfig::perception_config* config, const uint timestep);
  ~ntimestep_perception_subsystem(void) override;

  /* ntimestep perception metrics */
  uint n_known_blocks(void) const;  // can compare this stat with block objects in our object store??? ;
  uint n_known_caches(void) const;

  uint c_timestep(void) const; // timestep stuff here -- need to get this parameter through to nb_store

  /**
   * \brief Update the robot's perception of the environment, passing it its
   * current line of sight.
   */
  void update(oracular_info_receptor* receptor) override;

  /**
   * \brief Reset the robot's perception of the environment to an initial state
   */
  void reset(void) override;

  /**
   * const ds::dpo_store* dpo_store(void) const override { return m_store.get(); } //TODO: don't use dpo store
   * ds::dpo_store* dpo_store(void) override { return m_store.get(); }
   */ 

 private:
  /*
   * \brief Update the perceived arena map with the current line-of-sight,
   * update the relevance of information (density) within it (blocks and
   * caches).
   *
   * \param c_los The LOS to process.
   */
  void process_los(const repr::forager_los* c_los,
                   oracular_info_receptor* receptor);

  void process_los_blocks(const repr::forager_los* c_los);
  void process_los_caches(const repr::forager_los* c_los);

  void los_tracking_sync(const repr::forager_los* c_los,
                         const cads::bcache_vectorno& los_caches);
  void los_tracking_sync(const repr::forager_los* c_los,
                         const cds::block3D_vectorno& los_blocks);

  /* clang-format off */
  std::unique_ptr<ds::nb_store> m_store;
  /* clang-format on */
};

NS_END(cognitive, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_COGNITIVE_NTIMESTEP_PERCEPTION_SUBSYSTEM_HPP_ */
