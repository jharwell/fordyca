/**
 * @file dp_entity.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_DP_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_DP_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/swarm/pheromone_density.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

namespace rswarm = rcppsw::swarm;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @struct dp_entity
 * @ingroup representation
 *
 * @brief A representation of a Decaying Pheromone (DP) entity in the arena,
 * which has a pheromone density/relevance associated with it.
 *
 * When performing equality tests between instances, only the underlying entity
 * is considered (relevance is ignored).
 */
template<class T>
class dp_entity {
 public:
  dp_entity(const std::shared_ptr<T>& ent,
             const rcppsw::swarm::pheromone_density& density)
      : m_ent(ent), m_density(density) {}

  /**
   * @brief Compare two entities for equality. We must explicitly invoke
   * operator== on the object managed by the shared_ptr, otherwise we get only
   * pointer comparison, which is NOT what we want.
   */
  bool operator==(const dp_entity<T>& other) const {
    return m_ent->idcmp(*other.ent());
  }

  /*
   * Needed for placement into std::set. Since m_ent is a std::shared_ptr, it
   * will use direct pointer comparison, which is fine because we don't care.
   */
  bool operator<(const dp_entity<T>& other) const {
    return m_ent < other.m_ent;
  }
  const std::shared_ptr<T>& ent_obj(void) const { return m_ent; }

  T* ent(void) { return m_ent.get(); }
  const T* ent(void) const { return m_ent.get(); }

  const rswarm::pheromone_density& density(void) const { return m_density; }
  rswarm::pheromone_density& density(void) { return m_density; }

  void update(void) { m_density.update(); }

  void density(const rswarm::pheromone_density& density) { m_density = density; }


  /* clang-format on */
  std::shared_ptr<T>        m_ent;
  rswarm::pheromone_density m_density;
  /* clang-format off */
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_DP_ENTITY_HPP_ */
