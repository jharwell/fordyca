/**
 * @file loop_functions_utils.hpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/loop_functions_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, utils);

/*******************************************************************************
 * Functions
 ******************************************************************************/
int robot_on_block(const argos::CFootBotEntity &robot,
                   representation::arena_map &map) {
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity &>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetX(),
          const_cast<argos::CFootBotEntity &>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetY());
  return map.robot_on_block(pos);
} /* robot_on_block() */

int robot_id(const argos::CFootBotEntity &robot) {
  /* +2 because the ID string starts with 'fb' */
  return std::atoi(robot.GetId().c_str() + 2);
} /* robot_id() */

int robot_on_cache(const argos::CFootBotEntity &robot,
                   representation::arena_map &map) {
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity &>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetX(),
          const_cast<argos::CFootBotEntity &>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetY());
  return map.robot_on_cache(pos);
}

NS_END(utils, support, fordyca);
