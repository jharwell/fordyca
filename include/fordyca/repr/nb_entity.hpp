/**
 * \file nb_entity.hpp
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

#ifndef INCLUDE_FORDYCA_REPR_NB_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPR_NB_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nb_entity
 * \ingroup repr
 *
 * \brief A repr of a Decaying Pheromone (DP) entity in the arena,
 * which has a pheromone density/relevance associated with it.
 *
 * When performing equality tests between instances, only the underlying entity
 * is considered (relevance is ignored).
 */
template <class T>
class nb_entity {
 public:
  nb_entity(void) = default;
  nb_entity(std::unique_ptr<T> ent)
      : m_ent(std::move(ent)) {}

  /**
   * \brief Compare two entities for equality. We must explicitly invoke
   * operator== on the object managed by the unique_ptr, otherwise we get only
   * pointer comparison, which is NOT what we want.
   */
  bool operator==(const nb_entity<T>& other) const {
    return m_ent->idcmp(*other.ent());
  }

  T* ent(void) { return m_ent.get(); }
  const T* ent(void) const { return m_ent.get(); }

 private:
  /* clang-format off */
  std::unique_ptr<T>       m_ent;
  /* clang-format on */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_NB_ENTITY_HPP_ */
