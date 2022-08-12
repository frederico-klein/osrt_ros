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
 */
#include "osrt_ros/UIMU/UIMUInputDriver.h"
#include "Exception.h"
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "osrt_ros/UIMU/TfServer.h"
#include "osrt_ros/UIMU/CometaServer.h"

using namespace OpenSimRT;
using namespace SimTK;

UIMUInputDriver::UIMUInputDriver(const double& sendRate)
        : terminationFlag(false), rate(sendRate) {
        imu_names = {"thorax", "humerus", "radius" };
	server = new TfServer(imu_names);	
	ROS_WARN("IMU names are hardcoded!!! This needs to be solved or saving the TimeSeriesTable as CSV will be wrong!");
        }
UIMUInputDriver::UIMUInputDriver(const int port,
                                                   const double& sendRate)
        : terminationFlag(false), rate(sendRate) {
        server = new CometaServer(port, 4096);

	imu_names = {"thorax", "humerus", "radius" };
	ROS_WARN("IMU names are hardcoded!!! This needs to be solved or saving the TimeSeriesTable as CSV will be wrong!");

        }
UIMUInputDriver::UIMUInputDriver(std::vector<std::string> imuObservationOrder, const double& sendRate)
        : terminationFlag(false), rate(sendRate) {
        imu_names = imuObservationOrder;
	server = new TfServer(imu_names);	
        }

        // i maybe want to start the server!

UIMUInputDriver::~UIMUInputDriver() { t.join(); }

void UIMUInputDriver::startListening() {
    static auto f = [&]() {
        try {
            int i = 0;
            std::cout << "Rate: " << rate << std::endl ;
            for (;;) {
                if (shouldTerminate())
                    THROW_EXCEPTION("??? this is not great. File stream terminated.");
                {
                    std::lock_guard<std::mutex> lock(mu);
                    // get something from the udp stream
                    //TODO: rosdebug
		    ROS_DEBUG_STREAM( "Acquired lock. receiving.");
                    if (! server->receive()){
                            ROS_INFO_STREAM( "Received goodbye message!" );
                            terminationFlag = true;
                            break;
                    }
                    std::vector<double> output = server->output;
                    ROS_DEBUG_STREAM("Received.");

                    // there is no table, so this will be empty
                    //std::stringstream s(server.buffer);
                    //time = output[0]; // probably a double
                    //SimTK::readUnformatted<SimTK::Vector>(s, frame);// I will keep

		    ROS_DEBUG_STREAM("read input size from OrientationProvider" << output.size());
                    table.appendRow(output[0], output.begin()+1, output.end()); // superflex!
		    ROS_DEBUG_STREAM( "added to table alright." );
		    //table.getMatrix()[0]; // OpenSim::TimeSeriesTable
                //this will crash because table was not initialized.
                    time = table.getIndependentColumn()[i];
                    frame = table.getMatrix()[i];
                    newRow = true;
                    i++;
                }
                cond.notify_one();

                // artificial delay
                // maybe i don't need this.
                std::this_thread::sleep_for(std::chrono::milliseconds(
                        static_cast<int>(1 / rate * 1000)));
            }
            terminationFlag = true;
            cond.notify_one();

        } catch (const std::exception& e) {
            std::cout << "Failed in acquiring thread." << e.what() << std::endl;

            terminationFlag = true;
            cond.notify_one();
        }
    };
    t = std::thread(f);
    // this is threaded, so this guy should work too!
    std::cout << "Acquisition thread ended!" << std::endl;
}

bool UIMUInputDriver::shouldTerminate() {
    return terminationFlag.load();
}

void UIMUInputDriver::shouldTerminate(bool flag) {
    terminationFlag = flag;
    cond.notify_one();
}

UIMUInputDriver::IMUDataList
UIMUInputDriver::fromVector(const Vector& v) const {
    IMUDataList list;
    UIMUData data;
    int n = UIMUData::size();

    //std::cout << v << std::endl;
    //std::cout << "size of UIMUData: " << n  << std::endl;
    //std::cout << "size of v: " << v.size() << std::endl;

    for (int i = 0; i < v.size(); i += n) {
        data.fromVector(v(i, n));
        list.push_back(data);
        //i have 8 right now so this should tell me 8
        //std::cout << i << "number of vectors I pushed back" << std::endl;
    }
    return list;
}

UIMUInputDriver::IMUDataList
UIMUInputDriver::getData() const {
    std::unique_lock<std::mutex> lock(mu);
    cond.wait(lock,
              [&]() { return (newRow == true) || terminationFlag.load(); });
    newRow = false;
    return fromVector(frame.getAsVector());
}

std::pair<double, std::vector<UIMUData>> UIMUInputDriver::getFrame() {
    auto temp = getFrameAsVector();
    return std::make_pair(temp.first, fromVector(temp.second));
}

std::pair<double, Vector> UIMUInputDriver::getFrameAsVector() const {
    std::unique_lock<std::mutex> lock(mu);
    cond.wait(lock,
              [&]() { return (newRow == true) || terminationFlag.load(); });
    newRow = false;

    return std::make_pair(time, frame.getAsVector());
}

OpenSim::TimeSeriesTable UIMUInputDriver::initializeLogger() const {
        std::vector<std::string> suffixes = {
            "_q1",       "_q2",       "_q3",      "_q4",        "_ax",
            "_ay",       "_az",       "_gx",      "_gy",        "_gz",
            "_mx",       "_my",       "_mz",      "_barometer", "_linAcc_x",
            "_linAcc_y", "_linAcc_z", "_altitude"};

    // create column names for each combination of imu names and measurement
    // suffixes
        std::vector<std::string> columnNames;
    for (const auto& imu : imu_names) {
        for (const auto& suffix : suffixes) {
            columnNames.push_back(imu + suffix);
        }
    }

    // return table
    OpenSim::TimeSeriesTable q;
    q.setColumnLabels(columnNames);
    return q;
}

