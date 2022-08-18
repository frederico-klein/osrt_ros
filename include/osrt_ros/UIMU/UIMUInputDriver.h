/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 *
 * @file UIMUInputDriver.h
 *
 * @brief Concrete implementation of the IMUInputDriver to stream UIMU data
 * from file in a separate thread.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once
#include "osrt_ros/UIMU/InputDriver.h"
#include "osrt_ros/UIMU/UIMUData.h"
#include <Common/TimeSeriesTable.h>
#include <condition_variable>
#include <thread>
#include "osrt_ros/UIMU/OrientationProvider.h"
#include <vector>

namespace OpenSimRT {

/**
 * @brief xio UIMU Input driver for streaming data from file.
 */
class  UIMUInputDriver : public InputDriver<UIMUData> {
 public:
    /**
     * Create a UIMU driver that streams data from file at a constant rate.
     */
    UIMUInputDriver(const double& sendRate = 50);
    UIMUInputDriver(const int port,
                             const double& sendRate);
    UIMUInputDriver(const std::vector<std::string> imuObservationOrder,const double& sendRate = 50);
    ~UIMUInputDriver(); // dtor

    /**
     * Implements the startListening of the base class. Create a thread that
     * streams the data from file at a constant rate.
     */
    virtual void startListening() override;

    /**
     * Determine if the stream from file has ended.
     */
    bool shouldTerminate();

    /**
     * Set the termination flag;
     */
    void shouldTerminate(bool flag);

    /**
     * Get data from file as a list of UIMUData. Implements the stopListening
     * of the base class.
     */
    virtual IMUDataList getData() const override;

    /**
     * Get data from file as a std::pair containing the time and all the sensor
     * values from the table as a std::vector<UIMUData>. (i.e., time and row
     * from table)
     */
    std::pair<double, IMUDataList> getFrame();

    /**
     * Get data from file as a std::pair containing the time and all the sensor
     * values from the table as a SimTK::Vector. (i.e., time and row from table)
     */
    std::pair<double, SimTK::Vector> getFrameAsVector() const;

    // Using orientation provider pointer to choose server implementation
    OrientationProvider* server;

    OpenSim::TimeSeriesTable initializeLogger() const;

    // buffers
    SimTK::RowVector frame;
    double time;

    std::vector<std::string> imu_names;

 protected:
    /**
     * Reconstruct a list of UIMU from a SimTK::Vector.
     */
    IMUDataList fromVector(const SimTK::Vector&) const;

    // hide it from public since it does nothing
    void stopListening() override {}

 private:
    OpenSim::TimeSeriesTable table;
    //std::vector<double> output;

    double rate;


    // thread related variables
    std::thread t;
    mutable std::mutex mu;
    mutable std::condition_variable cond;
    std::atomic_bool terminationFlag;
    mutable bool newRow = false;
};
} // namespace OpenSimRT

