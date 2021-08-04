/**
 * \file dpo_map.hpp
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

#ifndef INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DS_DPO_MAP_HPP_
#define INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DS_DPO_MAP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm.hpp>
#include <map>
#include <utility>

#include "fordyca/repr/dpo_entity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_map
 * \ingroup subsystem perception ds
 *
 * \brief The Decaying Pheromone Object (DPO) map stores objects in the arena
 * SEPARATELY from the \ref arena_map where they actually live (clone not
 * reference), which decouples/simplifies a lot of the tricky handshaking logic
 * when robots interact with the arena.
 */
template <typename TKeyType,
          typename TRawValueType>
class dpo_map {
 public:
  using raw_value_type = TRawValueType;
  using value_type = typename repr::dpo_entity<TRawValueType>;
  using key_type = TKeyType;
  using map_type = std::map<key_type, value_type>;

  template<typename TRawVectorType>
  static TRawVectorType raw_values_extract(const dpo_map& map) {
    auto range = map.values_range();
    TRawVectorType ret;
    boost::range::for_each(range, [&](value_type& v) { ret.push_back(v.ent()); });
    return ret;
  }

  dpo_map(void)  = default;
  virtual ~dpo_map(void) = default;

  /**
   * \brief Update the densities of all objects in the map. Should be called
   * when one unit of time has passed (e.g. every timestep).
   */
  void decay_all(void) {
    for (auto& o : m_obj) {
      o.second.density().update();
    } /* for(&o..) */
  }

  /**
   * \brief Returns a pointer to the object that matches the specified key, or
   * nullptr if the key is not found in themap.
   */
  const value_type* find(const key_type& key) const RCPPSW_PURE {
    auto it = m_obj.find(key);
    return (it == m_obj.end()) ? nullptr : &(it->second);
  }
  value_type* find(const key_type& key) RCPPSW_PURE {
    auto it = m_obj.find(key);
    return (it == m_obj.end()) ? nullptr : &(it->second);
  }

  /**
   * \brief Returns \c TRUE iff the key is contained in the map, and \c FALSE
   * otherwise.
   */
  RCPPSW_PURE bool contains(const key_type& key) const {
    return nullptr != find(key);
  }

  /**
   * \brief Add the specified object from the map of known objects of that
   * type. If it is already in the map of known objects of that type, the old
   * version is replaced.
   */
  void obj_add(std::pair<key_type, value_type>&& obj) {
    m_obj.erase(obj.first);
    m_obj.insert(std::move(obj));
  }

  /**
   * \brief Return an iterator for examining, but not modifying the values of
   * the map.
   */
  auto values_range(void) const {
    return const_cast<dpo_map*>(this)->values_range();
  }

  auto values_range(void) {
    return m_obj | boost::adaptors::map_values;
  }

  /**
   * \brief Return an iterator for examining, but not modifying, the keys of
   * the map.
   */
  auto keys_range(void) const {
    return m_obj | boost::adaptors::map_keys;
  }

  /**
   * \brief Remove the specified object from the map of known objects of that
   * type (if it exists). If the argument is not in the map of known objects of
   * that type, no action is performed.
   */
  void obj_remove(const key_type& key) { m_obj.erase(key); }

 private:
  /* clang-format off */
  map_type        m_obj{};
  /* clang-format on */

 public:
  RCPPSW_WRAP_DECLDEF(size, m_obj, const)
  RCPPSW_WRAP_DECLDEF(empty, m_obj, const)
  RCPPSW_WRAP_DECLDEF(clear, m_obj)
};

NS_END(ds, perception, subsystem, fordyca);

#endif /* INCLUDE_FORDYCA_SUBSYSTEM_PERCEPTION_DS_DPO_MAP_HPP_ */
