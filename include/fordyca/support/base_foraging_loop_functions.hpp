/**
 * @file base_foraging_loop_functions.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BASE_FORAGING_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BASE_FORAGING_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <string>

#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/support/loop_functions_utils.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class base_foraging_loop_functions
 * @ingroup support
 *
 * @brief The base loop functions in FORDYCA that all other loop functions
 * inherit from.
 *
 * This class is not a functional set of loop functions, but it provides
 * functions needed across multiple derived classes, but functionality that
 * could not just be free functions because they require access to members in
 * the \ref argos::CLoopFunctions class.
 */
class base_foraging_loop_functions : public argos::CLoopFunctions {
 public:
  base_foraging_loop_functions(void) = default;

  base_foraging_loop_functions(const base_foraging_loop_functions& s) = delete;
  base_foraging_loop_functions& operator=(
      const base_foraging_loop_functions& s) = delete;

  void Init(ticpp::Element&) override {
    m_floor = &GetSpace().GetFloorEntity();
    std::srand(std::time(nullptr));
  }

 protected:
  argos::CFloorEntity* floor(void) const { return m_floor; }

 private:
  argos::CFloorEntity* m_floor{nullptr};
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BASE_FORAGING_LOOP_FUNCTIONS_HPP_ */
