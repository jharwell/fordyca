/**
 * \file block_selector.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "fordyca/controller/cognitive/block_sel_matrix.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(fordyca, controller, cognitive);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_selector
 * \ingroup controller cognitive d0
 *
 * \brief Select the best block that a robot knows about, for use in acquiring a
 * block as part of a higher level FSM.
 */
class block_selector : public rer::client<block_selector> {
 public:
  explicit block_selector(const block_sel_matrix* sel_matrix);

  ~block_selector(void) override = default;

  block_selector& operator=(const block_selector&) = delete;
  block_selector(const block_selector&) = delete;
  /**
   * \brief Given a list of blocks that a robot knows about (i.e. have not faded
   * into an unknown state), compute which is the "best", for use in deciding
   * which block to go attempt to pickup.
   *
   * \return A pointer to the "best" block, along with its utility value, if a
   * best block is found, and NULL.
   */
  const crepr::base_block3D* operator()(const fspds::dp_block_map& blocks,
                                        const rmath::vector2d& position);

 private:
  /**
   * \brief Determine if the specified block is excluded from being considered
   * for selection because:
   *
   * - The robot is too close to it (within block_dim meters of it). Allowing
   * robots to consider ANY block, regardless of how close it is to the robot,
   * can potentially get the robot stuck in an infinite loop of trying to
   * acquire a block that is REALLY close to it and failing, due to kinematic
   * parameters making its turning radius too large.
   *
   * - It is on the exception list.
   *
   * \return \c TRUE if the cache should be excluded, \c FALSE otherwise.
   */
  bool block_is_excluded(const rmath::vector2d& position,
                         const crepr::base_block3D* block) const;

  /* clang-format off */
  const block_sel_matrix* const mc_matrix;
  /* clang-format on */
};

NS_END(cognitive, fordyca, controller);

