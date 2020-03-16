/**
 * \file base_metrics_aggregator.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_BASE_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_METRICS_BASE_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <map>
#include <string>
#include <typeindex>
#include <utility>
#include <filesystem>

#include "rcppsw/er/client.hpp"
#include "rcppsw/metrics/collector_group.hpp"

#include "cosm/metrics/config/metrics_config.hpp"
#include "cosm/repr/base_block2D.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds::config {
struct grid_config;
} // namespace cosm::ds::config

NS_START(fordyca);

namespace support {
class base_loop_functions;
}
namespace controller {
class foraging_controller;
} /* namespace controller */

NS_START(metrics);
namespace fs = std::filesystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_metrics_aggregator
 * \ingroup metrics
 *
 * \brief Base class for aggregating collection of metrics for various
 * sources. Extends \ref rmetrics::collector_group to include
 * initialization bits to make loop functions simpler/clearer.
 */
class base_metrics_aggregator : public rer::client<base_metrics_aggregator> {
 public:
  base_metrics_aggregator(const cmconfig::metrics_config* mconfig,
                          const cdconfig::grid_config* const gconfig,
                          const std::string& output_root);
  ~base_metrics_aggregator(void) override = default;

  void collect_from_loop(const support::base_loop_functions* loop);

  /**
   * \brief Collect metrics from a block right before it is dropped in the nest.
   */
  void collect_from_block(const crepr::base_block2D* block);

  /**
   * \brief Collect metrics from \ref foraging_controller.
   */
  void collect_from_controller(const controller::foraging_controller* controller);

  const fs::path& metrics_path(void) const { return m_metrics_path; }

  /**
   * \brief To be called before \ref collector_register(), in order to correctly
   * set up the collector map for the collector with the specified scoped
   * name.
   *
   * If you forget to do this you will get a segfault.
   */
  void collector_preregister(const std::string& scoped_name,
                             rmetrics::output_mode mode) {
    if (rmetrics::output_mode::ekAPPEND == mode) {
      m_collector_map[scoped_name] = &m_append;
    } else if (rmetrics::output_mode::ekTRUNCATE == mode) {
      m_collector_map[scoped_name] = &m_truncate;
    } else if (rmetrics::output_mode::ekCREATE == mode) {
      m_collector_map[scoped_name] = &m_create;
    }
  }

  /**
   * \brief Decorator around \ref collector_group::collector_register().
   */
  template <typename TCollectorType, typename... Args>
  bool collector_register(const std::string& scoped_name,
                          const std::string& fpath,
                          Args&&... args) {
    return m_collector_map[scoped_name]->collector_register<TCollectorType>(
        scoped_name, fpath, std::forward<Args>(args)...);
  }

  void reset_all(void) {
    m_create.reset_all();
    m_append.reset_all();
    m_truncate.reset_all();
  }

  /**
   * \brief Decorator around \ref collector_group::collect().
   */
  template <typename T>
  void collect(const std::string& scoped_name, const T& collectee) {
    auto it = m_collector_map.find(scoped_name);
    if (it != m_collector_map.end()) {
      it->second->collect(scoped_name, collectee);
    }
  } /* collect() */

  /**
   * \brief Decorator around \ref collector_group::collect_if().
   */
  template <typename T>
  void collect_if(const std::string& scoped_name,
                  const T& collectee,
                  const std::function<bool(const rmetrics::base_metrics&)>& pred) {
    auto it = m_collector_map.find(scoped_name);
    if (it != m_collector_map.end()) {
      it->second->collect_if(scoped_name, collectee, pred);
    }
  } /* collect() */

  /**
   * \brief Decorator around \ref collector_group::collector_remove().
   */
  bool collector_remove(const std::string& scoped_name) {
    auto it = m_collector_map.find(scoped_name);
    if (it != m_collector_map.end()) {
      return it->second->collector_remove(scoped_name);
    }
    return false;
  }

  /**
   * \brief Decorator around \ref collector_group::get().
   */
  template <typename T>
  const T* get(const std::string& key) {
    return m_collector_map[key]->get<T>(key);
  }

  bool metrics_write(rmetrics::output_mode mode) {
    if (rmetrics::output_mode::ekAPPEND == mode) {
      return m_append.metrics_write_all();
    } else if (rmetrics::output_mode::ekTRUNCATE == mode) {
      return m_truncate.metrics_write_all();
    } else if (rmetrics::output_mode::ekCREATE == mode) {
      return m_create.metrics_write_all();
    }
    return false;
  }

  /**
   * \brief Decorator around \ref collector_group::timestep_inc_all().
   */
  void timestep_inc_all(void) {
    m_append.timestep_inc_all();
    m_truncate.timestep_inc_all();
    m_create.timestep_inc_all();
  }

  /**
   * \brief Decorator around \ref collector_group::interval_reset_all().
   */
  void interval_reset_all(void) {
    m_append.interval_reset_all();
    m_truncate.interval_reset_all();
    m_create.interval_reset_all();
  }

  /**
   * \brief Decorator around \ref collector_group::finalize_all().
   */
  void finalize_all(void) {
    m_append.finalize_all();
    m_truncate.finalize_all();
    m_create.finalize_all();
  }

 private:
  using collector_map_type = std::map<std::string, rmetrics::collector_group*>;

  /* clang-format off */
  fs::path                  m_metrics_path;
  collector_map_type        m_collector_map{};
  rmetrics::collector_group m_append{};
  rmetrics::collector_group m_truncate{};
  rmetrics::collector_group m_create{};
  /* clang-format on */
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_BASE_METRICS_AGGREGATOR_HPP_ */
