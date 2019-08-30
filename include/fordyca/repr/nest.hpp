/**
 * @file nest.hpp
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

#ifndef INCLUDE_FORDYCA_REPR_NEST_HPP_
#define INCLUDE_FORDYCA_REPR_NEST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include <argos3/plugins/simulator/entities/light_entity.h>

#include "rcppsw/types/discretize_ratio.hpp"

#include "fordyca/repr/colored_entity.hpp"
#include "fordyca/repr/unicell_immovable_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class nest
 * @ingroup fordyca repr
 *
 * @brief Class representing the nest in the arena, which is multi-cellular and
 * immobile.
 *
 * When initializing lights, they will only be detectable by the footbot's light
 * sensor, NOT the omnidirectional camera, as that can only detect LED entities
 * that are on the ground (implementation detail).
 */
class nest : public unicell_immovable_entity, public colored_entity {
 public:
  /**
   * @brief We use raw pointers to indicate we (FORDYCA) do not own the
   * constructed lights. If we own them, then when ARGoS goes to delete them
   * after the experiment has ended the arena has already been deconstructed and
   * the nest lights along with them, and an exception is thrown.
   */
  using light_list = std::list<argos::CLightEntity*>;

  nest(const rmath::vector2d& dim,
       const rmath::vector2d& loc,
       rtypes::discretize_ratio resolution,
       const rutils::color& light_color);

  /**
   * @brief Determine if a real-valued point lies within the extent of the
   * nest for:
   *
   * 1. Visualization purposes.
   * 2. Determining if a robot has entered the nest.
   *
   * @param point The point to check.
   *
   * @return \c TRUE if the condition is met, and \c FALSE otherwise.
   */
  bool contains_point(const rmath::vector2d& point) const {
    return xspan().contains(point.x()) && yspan().contains(point.y());
  }

  light_list& lights(void) { return m_lights; }

 private:
  light_list init_lights(const rutils::color& color) const;
  light_list init_square(const rutils::color& color) const;
  light_list init_rect(const rutils::color& color) const;

  /* clang-format off */
  light_list m_lights;
  /* clang-format on */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_NEST_HPP_ */
