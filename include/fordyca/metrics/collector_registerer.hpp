/**
 * \file collector_registerer.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_METRICS_COLLECTOR_REGISTERER_HPP_
#define INCLUDE_FORDYCA_METRICS_COLLECTOR_REGISTERER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <set>
#include <string>
#include <tuple>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/ds/config/grid_config.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/metrics/base_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class collector_registerer
 * \ingroup metrics
 *
 * \brief Register all enabled collectors with an aggregator.
 */

class collector_registerer : public rer::client<collector_registerer> {
 public:
  /**
   * \brief Collectors are not default constructible, and therefore cannot be
   * used with boost::mpl::for_each as is. But because all we need is the type
   * of the collector during registration in order to construct it, and do not
   * need access to any of its methods, we can add a layer of indirection with a
   * simple struct that IS default constructible that we CAN use.
   */
  template <typename T>
  struct type_wrap {
    using type = T;
  };

  /**
   * \brief Each entry in the set of collectors that CAN be created (if they are
   * actually created dependent on configuration) has:
   *
   * - The typeid of the collector, so that functions templated on collector
   *   type can figure out the correct item in the set to read from.
   *
   * - The name of the collector in the input src (e.g. the XML attribute name)
   *
   * - The scoped name of the collector that will be used to refer to the
   *   created collector during simulation.
   */
  using set_value_type = std::tuple<std::type_index, std::string, std::string>;

  /**
   * \brief Comparator for \ref set_value_type objects within the \ref
   * creatable_set.
   */
  struct set_comparator {
    bool operator()(const set_value_type& lhs, const set_value_type& rhs) const {
      return std::get<0>(lhs) < std::get<0>(rhs);
    }
  };

  /**
   * \brief Set of \ref set_value_types in which duplicates are allowed, because
   * when we compare elements, we only use the typeid as the key, which can be
   * the same between collectors, even if the other parts of each element are
   * different.
   */
  using creatable_set = std::multiset<set_value_type, set_comparator>;

  /**
   * \brief A collector is constructible using the expected function arguments.
   */
  template <typename T>
  using expected_constructible = std::is_constructible<T,
                                                       const std::string&,
                                                       const rtypes::timestep&>;

  /**
   * \brief Some metrics collectors (e.g. \ref
   * temporal_variance_metrics_collector) do not require the collection interval
   * argument to their constructor, as they MUST be gathered every timestep,
   * regardless of configuration.
   */
  template <typename T>
  using constructible_without_collect_interval =
      std::is_constructible<T, const std::string&>;

  /**
   * \brief Some metrics collectors (e.g. \ref collision_locs_metrics_collector)
   * require the arena dimensions as an argument to their constructor.
   */
  template <typename T>
  using constructible_with_arena_dim =
      std::is_constructible<T,
                            const std::string&,
                            const rtypes::timestep&,
                            const rmath::vector2u&>;

  /**
   * \brief Some metrics collectors (e.g. \ref bi_tdgraph_metrics_collector)
   * require an additional integer as an argument to their constructor.
   */
  template <typename T>
  using constructible_with_uint =
      std::is_constructible<T,
                            const std::string&,
                            const rtypes::timestep&,
                            uint>;

  /**
   * \brief Initialize the registerer.
   *
   * \param config Metrics configuration, specifying which metrics should be
   *               collected.
   *
   * \param agg The metrics aggregator to register the collectors with.
   *
   * \param create_set Definitions for all the possible collectors to create.
   * \param decomposition_depth The height of the complete binary tree formed by
   *                           the task decomposition graph.
   *
   * Both of the maps are necessary to provide an input src agnostic means of
   * mapping collectors to run-time categories, so that this class is general
   * purpose and not tied to a specific input format.
   */
  collector_registerer(const cmconfig::metrics_config* const config,
                       const cdconfig::grid_config* const grid_config,
                       const creatable_set& create_set,
                       base_metrics_aggregator* const agg,
                       int decomposition_depth = -1)
      : ER_CLIENT_INIT("fordyca.metrics.collector_registerer"),
        mc_decomp_depth(decomposition_depth),
        mc_arena_dim(
            rmath::dvec2uvec(grid_config->upper, grid_config->resolution.v())),
        mc_config(config),
        mc_create_set(create_set),
        m_agg(agg) {}
  collector_registerer(const collector_registerer&) = default;
  collector_registerer operator=(const collector_registerer&) = delete;

  template <typename TCollectorWrap>
  void operator()(const TCollectorWrap&) const {
    std::type_index id(typeid(typename TCollectorWrap::type));

    auto range = mc_create_set.equal_range(set_value_type(id, "", ""));

    ER_ASSERT(mc_create_set.end() !=
                  mc_create_set.find(set_value_type(id, "", "")),
              "Unknown collector: type_index='%s'",
              id.name());

    /*
     * Multiple collectors of the same type can be registered as different
     * scoped/runtime names, so we need to iterate.
     */
    for (auto it = range.first; it != range.second; ++it) {
      std::string fpath =
          collector_fpath_create(mc_config->enabled, std::get<1>(*it));

      if (!fpath.empty()) {
        bool ret = do_register<TCollectorWrap>(std::get<2>(*it), fpath);
        if (!ret) {
          ER_WARN("Collector with scoped_name='%s' already exists!",
                  std::get<2>(*it).c_str());
        } else {
          ER_INFO("Metrics enabled: xml_name='%s',scoped_name='%s'",
                  std::get<1>(*it).c_str(),
                  std::get<2>(*it).c_str());
        }
      }
    } /* for(it..) */
  }

 private:
  template <typename TCollectorWrap,
            RCPPSW_SFINAE_FUNC(constructible_with_arena_dim<
                               typename TCollectorWrap::type>::value)>
  bool do_register(const std::string& scoped_name,
                   const std::string& fpath) const {
    return m_agg->collector_register<typename TCollectorWrap::type>(
        scoped_name, fpath, mc_config->output_interval, mc_arena_dim);
  }

  template <typename TCollectorWrap,
            RCPPSW_SFINAE_FUNC(
                expected_constructible<typename TCollectorWrap::type>::value)>
  bool do_register(const std::string& scoped_name,
                   const std::string& fpath) const {
    return m_agg->collector_register<typename TCollectorWrap::type>(
        scoped_name, fpath, mc_config->output_interval);
  }

  template <typename TCollectorWrap,
            RCPPSW_SFINAE_FUNC(constructible_without_collect_interval<
                               typename TCollectorWrap::type>::value)>
  bool do_register(const std::string& scoped_name,
                   const std::string& fpath) const {
    return m_agg->collector_register<typename TCollectorWrap::type>(scoped_name,
                                                                    fpath);
  }

  template <typename TCollectorWrap,
            RCPPSW_SFINAE_FUNC(
                constructible_with_uint<typename TCollectorWrap::type>::value)>
  bool do_register(const std::string& scoped_name,
                   const std::string& fpath) const {
    return m_agg->collector_register<typename TCollectorWrap::type>(
        scoped_name, fpath, mc_config->output_interval, mc_decomp_depth);
  }

  /**
   * \brief Return the output filename that should be associated with a
   * collector if it is enabled, and "" otherwise.
   */
  std::string collector_fpath_create(
      const cmconfig::metrics_config::enabled_map_type& enabled,
      const std::string& collector_name) const {
    auto it = enabled.find(collector_name);
    return (it == enabled.end()) ? std::string()
                                 : m_agg->metrics_path() + "/" + it->second;
  }

  /* clang-format off */
  const int                             mc_decomp_depth;
  const rmath::vector2u                 mc_arena_dim;
  const cmconfig::metrics_config* const mc_config;
  const creatable_set                   mc_create_set;

  base_metrics_aggregator* const        m_agg;
  /* clang-format on */
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTOR_REGISTERER_HPP_ */
