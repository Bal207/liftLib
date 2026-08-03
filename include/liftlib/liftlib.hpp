#pragma once

#include "liftlib/lift.hpp"
#include "liftlib/motor_config.hpp"
#include "liftlib/pid.hpp"
#include "liftlib/subsystem.hpp"

#define LIFTLIB_VERSION_MAJOR 1
#define LIFTLIB_VERSION_MINOR 0
#define LIFTLIB_VERSION_PATCH 0
#define LIFTLIB_VERSION "1.0.0"

/**
 * Define LIFTLIB_NO_GLOBAL_NAMES before including this header to keep every
 * name inside the liftlib namespace. Without it the public types are also
 * reachable unqualified, which keeps existing code working.
 */
#ifndef LIFTLIB_NO_GLOBAL_NAMES
using liftlib::Lift;
using liftlib::MotorConfig;
using liftlib::PID;
using liftlib::Subsystem;
#endif
