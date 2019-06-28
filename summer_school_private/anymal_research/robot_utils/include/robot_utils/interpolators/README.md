# Interpolators

## Overview

Defines interpolation classes and utilities for performing and extending interpolators.

There are two kinds of classes defined here: Interpolation (ScalarInterpolation and derived classes) and Interpolators.

**Author(s):** Perry Franklin, Koen Kraemer

## ScalarInterpolation

Interpolation classes define the mathematical functions underlying the interpolation. These are functions that are defined as `f(t)->s`, `[0,1] -> [0,1]`, with behavior in `(-inf, 0]` and `[1.0, inf)` expected to be extrapolation (although not guaranteed, depending on the interpolation function in question). `f(0.0) = 0.0`, `f(1.0) = 1.0`. These should also define their derivatives with respect to the input variable.

### LinearInterpolation

Defines LinearInterpolation. In `[0,1]`, this is `s = t`, with `dsdt = 1`.

### BasicFifthOrderInterpolation

Defines a fifth-order polynomial interpolation, meaning that at `t = 0` or `1`, the first and second derivatives are `0`.

## Interpolator

Although not mandated by a abstract or templated class, Interpolators should provide interpolation for specific cases. As a constructor, they take a std::unique_ptr to the type of interpolation they should use.

Interpolators function in time, not in the interval `[0,1]`. On initialization, starting and ending values and a duration are given, and the time is reset to `0`.

To use this class, call `advance(double dt)` to move the time forward by dt (ie `time = time+dt`). Accessors for the current Value of the interpolation and the derivative are given. Call `isFinished()` to see if the interpolation is completed (`time >= duration`). `getProgress()` returns the percentage completed in the range `[0,1]`.

For `time < 0.0` and `time >= 0.0` , the interpolator is stationary, and the derviative is always `0`. At `0`, the derviative is not neccessarily `0`.

### GenericInterpolator

A templated class for providing interpolation to any linear type. Any type with `operator+(LinearType)` and `operator(LinearType)*` defined can be used, and will have the correct behavior if the type is linear.

### PoseInterpolator

A class for interpolating `kindr::Pose`. Special care must be taken for quaternions, hence its separation from GenericInterpolator.