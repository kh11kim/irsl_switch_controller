#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <mutex>
#include <functional>
#include <thread>
#include <chrono>

//libfranka
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>

//Kortex(kinova) API
#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <google/protobuf/util/json_util.h>

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef std::array<double, 7> Array7d;
inline void EigenToArray(const Vector7d &eigen_vector, Array7d &std_array){
  for (int i = 0; i < eigen_vector.size(); ++i) {
    std_array[i] = eigen_vector[i];
  }
}
inline void ArrayToEigen(const Array7d &std_array, Vector7d &eigen_vector){
  for (int i = 0; i < std_array.size(); ++i) {
    eigen_vector[i] = std_array[i];
  }
}