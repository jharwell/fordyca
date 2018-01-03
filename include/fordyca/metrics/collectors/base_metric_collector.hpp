/**
 * @file base_metric_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_COLLECTORS_BASE_METRIC_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_COLLECTORS_BASE_METRIC_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <fstream>
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

namespace collectible_metrics { class base_collectible_metrics; }

NS_START(collectors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_metric_collector
 * @ingroup metrics
 *
 * @brief Base class that uses the template design pattern to provide hooks for
 * derived classes so that the process of writing out metrics is centralized in
 * one place (here).
 *
 * Metrics are written out in .csv format at whatever frequency derived classes
 * choose.
 */
class base_metric_collector {
 public:
  base_metric_collector(const std::string& ofname, bool collect_cum) :
      m_collect_cum(collect_cum), m_use_interval(false), m_interval(-1),
      m_timestep(0), m_ofname(ofname), m_separator(";"),
      m_ofile() {}

  virtual ~base_metric_collector(void)  = default;

  /**
   * @brief Reset the metrics completely, as if none have yet been collected.
   *
   * Should be called only on collection start/reset.
   */
  virtual void reset(void);

  /**
   * @brief Reset metrics at the end of an interval.
   *
   * Can be called every timestep (and probably should be for consistency as
   * behavior of derived classes possibly changes), and metrics will only be
   * reset after the specified number of timesteps in the interval has elapsed.
  */
  void interval_reset(void);

  void timestep_inc(void) { ++m_timestep; }
  uint timestep(void) const { return m_timestep; }

  /**
   * @brief Reset metrics at the end of a timestep.
   *
   * Should be called every timestep (default implementation does nothing).
   */
  void timestep_reset(void) { reset_after_timestep(); }

  /**
   * @brief Write out the gathered metrics.
   *
   * @param timestep The current timestep.
   */
  void csv_line_write(uint timestep);

  /**
   * @brief Finalize metrics and flush files.
   */
  void finalize(void) { m_ofile.close(); }

  /**
   * @brief Enable/disable interval usage for the current collector.
   */
  void use_interval(bool use_interval) { m_use_interval = use_interval; }

  /**
   * @brief Set the interval (# timesteps) for the current collector.
   */
  void interval(int interval) { m_interval = interval; }
  int interval(void) const { return m_interval; }

 protected:
  /**
   * @brief Reset some metrics (possibly).
   *
   * Can be called at the end of every interval. By default it does nothing.
   */
  virtual void reset_after_interval(void) {}

  /**
   * @brief Reset some metrics (possibly).
   *
   * Can be called every timestep. By default it does nothing.
   */
  virtual void reset_after_timestep(void) {}

  /**
   * @brief Build the header line for a particular collector.
   *
   * The default one only contains a single column: the current timestep.
   *
   * @param header The current header.
   *
   * @return The built header.
   */
  virtual std::string csv_header_build(const std::string& header);

  /**
   * @brief Build the next line of metrics
   *
   * @param line The current line, to be filled.
   *
   * @return \c TRUE if the metrics should be written out, or \c FALSE if
   * not. This allows metrics to be gathered across multiple timesteps, but only
   * written out once an interesting event has occurred.
   */
  virtual bool csv_line_build(std::string& line) = 0;

  virtual void collect(const collectible_metrics::base_collectible_metrics& metrics) = 0;

  /**
   * @brief Write out the default header, which only contains "clock;"
   */
  void csv_header_write(void);

  const std::string& separator(void) const { return m_separator; }
  bool collect_cum(void) const { return m_collect_cum; }

 private:
  bool          m_collect_cum;
  bool          m_use_interval;
  int           m_interval;
  uint          m_timestep;
  std::string   m_ofname;
  std::string   m_separator;
  std::ofstream m_ofile;
};

NS_END(collectors, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTORS_BASE_METRIC_COLLECTOR_HPP_ */
