/**
 * @file dpo_set.hpp
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

#ifndef INCLUDE_FORDYCA_DS_DPO_SET_HPP_
#define INCLUDE_FORDYCA_DS_DPO_SET_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <set>
#include "fordyca/representation/dp_entity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class dpo_set
 * @ingroup ds
 *
 * @brief The Decaying Pheromone Object (DPO) set stores objects in the arena
 * SEPARATELY from the \ref arena_map where they actually live (clone not
 * reference), which decouples/simplifies a lot of the tricky handshaking logic
 * when robots interact with the arena.
 */
template<typename T>
class dpo_set {
 public:
  using value_type = typename representation::dp_entity<T>;

  /**
   * @brief Update the densities of all objects in the set. Should be called
   * when one unit of time has passed (e.g. every timestep).
   */
  void decay_all(void) {
    /*
     * Generally speaking it is not a good idea to use set iterators in a
     * non-const context, as modification to the underlying object has the
     * potential to invalidate the internal set sorting. BUT, in this case it is
     * OK, because we only modify the density, which is not part of the set
     * sorting (only the underlying entity is used).
     */
    for (auto &&o : m_obj) {
      const_cast<value_type&>(o).update();
    } /* for(&o..) */
  }

  const value_type* find(const value_type& obj) const {
    auto it = std::find(m_obj.begin(), m_obj.end(), obj);
    return (it == m_obj.end()) ? nullptr : &(*it);
  }

  bool contains(const value_type& obj) const {
    for (auto &&o : m_obj) {
      if (o == obj) {
        return true;
      }
    } /* for(&&o..) */
    return false;
  }
  /**
   * @brief Add the specified object from the set of known objects of that
   * type. If it is already in the set of known objects of that type, the old
   * version is replaced.
   */
  void obj_add(const value_type& obj) {
    auto r = m_obj.insert(obj);
    if (!r.second) {
      m_obj.erase(r.first);
      m_obj.insert(obj);
    }
  }

  /**
   * @brief Remove the specified object from the set of known objects of that
   * type (if it exists). If the argument is not in the set of known objects of
   * that type, no action is performed.
   */
  void obj_remove(const value_type& obj) {
    m_obj.erase(obj);
  }

 private:
  /* clang-format off */
  std::set<value_type> m_obj{};
  /* clang-format on */

 public:
  RCPPSW_WRAP_MEMFUNC(size, m_obj, const);
  RCPPSW_WRAP_MEMFUNC(empty, m_obj, const);
  RCPPSW_WRAP_MEMFUNC(begin, m_obj, const);
  RCPPSW_WRAP_MEMFUNC(end, m_obj, const);
  RCPPSW_WRAP_MEMFUNC(begin, m_obj);
  RCPPSW_WRAP_MEMFUNC(end, m_obj);
  RCPPSW_WRAP_MEMFUNC(clear, m_obj);
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_DPO_SET_HPP_ */
