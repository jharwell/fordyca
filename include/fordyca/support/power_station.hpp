/**
 * @file power_station.hpp
 *
 * @copyright 2018 Nimer WazWaz, All rights reserved.
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
#ifndef INCLUDE_FORDYCA_SUPPORT_POWER_STATION_HPP_
#define INCLUDE_FORDYCA_SUPPORT_POWER_STATION_HPP_


/*******************************************************************************
 * Namespaces/Includes
 ******************************************************************************/
#include "fordyca/representation/immovable_cell_entity.hpp"
#include "fordyca/representation/multicell_entity.hpp"
#include "core/simulator/loop_functions.h"
#include "plugins/simulator/entities/light_entity.h"
#include "core/utility/datatypes/color.h"

#include <string>
#include <vector>

NS_START(fordyca, representation);
/*******************************************************************************
 * Classes
 ******************************************************************************/
class power_station : public multicell_entity,
                      public immovable_cell_entity {

public:

  power_station(double dimension,
                double resolution,
                int power_station_id,
                argos::CVector2 center) : multicell_entity(rcppsw::math::vector2d(dimension, dimension),  rcppsw::utils::color::kGRAY40),
                immovable_cell_entity(center, resolution) {
                  id_ = power_station_id;
                  m_resolution_ = resolution;
                  argos::CVector3 light_entity_loc(center.GetX(), center.GetY(), 5) ;
                  power_station_light_ = new CLightEntity(std::to_string(id_),  argos::CColor::MAGENTA, light_entity_loc , 10);
                }


  int get_id() {return id}

  int get_size() {return size}

  int get_light_entity() {return &power_station_light_}

private:
  int id_{-1};
  double m_resolution_{0};
  argos::CLightEntity * power_station_light_;
};










ns_end(fordyca, representation);

#endif /* INCLUDE_FORDYCA_SUPPORT_POWER_STATION_HPP_ */
