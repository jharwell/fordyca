/**
 * \file dpo_entity.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/repr/pheromone_density.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_entity
 * \ingroup repr
 *
 * \brief A repr of a Decaying Pheromone (DP) entity in the arena,
 * which has a pheromone density/relevance associated with it.
 *
 * When performing equality tests between instances, only the underlying entity
 * is considered (relevance is ignored).
 */
template <class T>
class dpo_entity {
 public:
  dpo_entity(void) = default;
  dpo_entity(std::unique_ptr<T> ent, const crepr::pheromone_density& density)
      : m_ent(std::move(ent)), m_density(density) {}

  /**
   * \brief Compare two entities for equality. We must explicitly invoke
   * operator== on the object managed by the unique_ptr, otherwise we get only
   * pointer comparison, which is NOT what we want.
   */
  bool operator==(const dpo_entity<T>& other) const {
    return m_ent->idcmp(*other.ent());
  }

  T* ent(void) { return m_ent.get(); }
  const T* ent(void) const { return m_ent.get(); }

  const crepr::pheromone_density& density(void) const { return m_density; }
  crepr::pheromone_density& density(void) { return m_density; }

  void density(const crepr::pheromone_density& density) { m_density = density; }

 private:
  /* clang-format off */
  std::unique_ptr<T>       m_ent;
  crepr::pheromone_density m_density;
  /* clang-format on */
};

NS_END(repr, fordyca);

