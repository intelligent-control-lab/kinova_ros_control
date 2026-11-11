#pragma once

#include "spdlog/spdlog.h"

#include <BaseClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <BaseCyclicClientRpc.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <memory>
#include <mutex>
#include <vector>
#include <tuple>
#include <functional>
#include <stdexcept>
#include <math.h>
#include <chrono>
#include <algorithm>

#define DEG2RAD(angle) (static_cast<double>(angle)*M_PI/180.0)
#define RAD2DEG(angle) (static_cast<double>(angle)/M_PI*180.0)
