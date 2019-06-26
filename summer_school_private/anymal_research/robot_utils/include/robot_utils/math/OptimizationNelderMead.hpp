/*
 * OptimizationNelderMead.hpp
 *
 *  Created on: Jul 19, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

// c++
#include <algorithm>
#include <iostream>
#include <functional>
#include <vector>

// eigen
#include <Eigen/Core>


namespace robot_utils {

namespace optimization {

class OptimizationNelderMead
{
 private:
  using Fcn = std::function<double(const Eigen::VectorXd&)>;

  // Constants.
  const double rho_ = 1.0;
  const double chi_ = 2.0;
  const double psi_ = 0.5;
  const double sigma_ = 0.5;


  // Run time variables.
  long int numCoordinates_ = 0;
  long int numVertices_ = 0;
  long int s_ = 0;
  long int h_ = 0;

  Eigen::MatrixXd simplex_;
  std::vector<double> fValues_;

  unsigned int currentIteration_ = 0;


  // Optimization parameters.
  Eigen::VectorXd x0_;
  Fcn fcn_;
  bool minimize_ = true;
  unsigned int maxIterations_ = 0;
  double maxFValueDiff_ = 0.0;
  double maxVertexDiff_ = 0.0;


 public:
  OptimizationNelderMead() {}
  virtual ~OptimizationNelderMead() {}

  bool optimize(double& fOpt,
                Eigen::VectorXd& xOpt,
                const Eigen::VectorXd& x0,
                const Fcn& fcn,
                bool minimize = true,
                long int maxIterations = -1,
                double maxFValueDiff = 1e-4,
                double maxVertexDiff = 1e-4) {


    // Check Input.
    if (xOpt.size() != x0.size()) {
      std::cout << "Parameter vector (" << xOpt.size() << "x1) and initial guess (" << x0.size() << "x1) have different sizes!" << std::endl;
      return false;
    }

    // Set run time variables.
    numCoordinates_ = x0.size();
    numVertices_ = numCoordinates_+1;
    s_ = numVertices_-2;
    h_ = numVertices_-1;

    simplex_ = Eigen::MatrixXd::Zero(numCoordinates_, numVertices_);
    fValues_.resize(numVertices_, 0.0);

    currentIteration_ = 0;

    // Set optimization parameters.
    x0_ = x0;
    fcn_ = fcn;
    minimize_ = minimize;
    maxIterations_ = (maxIterations == -1) ? 200*numCoordinates_ : maxIterations;
    maxFValueDiff_ = maxFValueDiff;
    maxVertexDiff_ = maxVertexDiff;

    setupSimplex();

    while (!terminateOptimization()) {
      runMinimizationStep();
    }

    if (!minimize_) {
      fOpt = -fValues_[0];
    } else {
      fOpt = fValues_[0];
    }

    xOpt = simplex_.col(0);

    return true;
  }


 private:
  int getIterations() {
    return currentIteration_;
  }

  void setupSimplex() {
    // Set initial point
    simplex_.col(0) = x0_;
    fValues_[0] = evalFcn(x0_);

    // Setup other simplex points
    const double usual_d = 0.05;
    const double zero_term_d = 0.00025;

    for (int k=0; k<numCoordinates_; k++) {
      Eigen::VectorXd y = x0_;
      if (y[k] != 0 ) {
        y[k] = (1+usual_d)*y[k];
      } else {
        y[k] = zero_term_d;
      }

      simplex_.col(k+1) = y;
      fValues_[k+1] = evalFcn(y);
    }

    sortSimplex();
  }

  double evalFcn(const Eigen::VectorXd& x) {
    if (minimize_) return fcn_(x);
    else return -fcn_(x);
  }

  void runMinimizationStep() {

    // Compute the centroid of the best n points.
    Eigen::VectorXd c = Eigen::VectorXd::Zero(numCoordinates_);
    for (int k=0; k<h_; k++) {
      c += simplex_.col(k);
    }
    c /= h_;

    // Compute the reflection point.
    const Eigen::VectorXd xr = (1.0+rho_)*c - rho_*simplex_.col(h_);
    const double fr = evalFcn(xr);

    if (fr < fValues_[0]) {
      // Compute the expansion point.
      const Eigen::VectorXd xe = (1.0+rho_*chi_)*c - rho_*chi_*simplex_.col(h_);
      const double fe = evalFcn(xe);

      if (fe < fr) {
        // Expansion.
        simplex_.col(h_) = xe;
        fValues_[h_] = fe;
      } else {
        // Reflection.
        simplex_.col(h_) = xr;
        fValues_[h_] = fr;
      }

    } else {
      if (fr < fValues_[s_]) {
        // Reflection.
        simplex_.col(h_) = xr;
        fValues_[h_] = fr;
      } else {
        // Contraction.
        if (fr < fValues_[h_]) {
          const Eigen::VectorXd xc = (1.0+psi_*rho_)*c - psi_*rho_*simplex_.col(h_);
          const double fc = evalFcn(xc);

          if (fc <= fr) {
            // Outside contraction.
            simplex_.col(h_) = xc;
            fValues_[h_] = fc;
          } else {
            // Shrink.
            shrinkSimplex();
          }

        } else {
          const Eigen::VectorXd xcc = (1.0-psi_)*c + psi_*simplex_.col(h_);
          const double fcc = evalFcn(xcc);

          if (fcc < fValues_[h_]) {
            // Inside contraction.
            simplex_.col(h_) = xcc;
            fValues_[h_] = fcc;
          } else {
            // Shrink.
            shrinkSimplex();
          }
        }
      }
    }

    sortSimplex();

    currentIteration_++;

    //printSimplex();
  }

  bool terminateOptimization() {
    // Check for number of iterations.
    if (currentIteration_ >= maxIterations_) {
      return true;
    }

    // Check for relative difference between f values.
    double maxFValueDiff = 0.0;
    for (size_t k=1; k<fValues_.size(); k++) {
      maxFValueDiff = std::max(maxFValueDiff, std::fabs(fValues_[0]-fValues_[k]));
    }

    // Check for relative difference between vertices.
    Eigen::MatrixXd simplexFirstColCopy(numCoordinates_, numVertices_-1);
    simplexFirstColCopy.setZero();

    for (int k=0; k<simplexFirstColCopy.cols(); k++) {
      simplexFirstColCopy.col(k) = simplex_.col(0);
    }

    const double maxVertexDiff = ((simplex_.block(0, 1, numCoordinates_, numVertices_-1) - simplexFirstColCopy).cwiseAbs()).maxCoeff();

    // Check if smaller than tolerance.
    if (maxFValueDiff <= maxFValueDiff_ && maxVertexDiff <= maxVertexDiff_) {
      return true;
    }

    return false;
  }

  template <typename T>
  static std::vector<int> getSortedIndexes(const std::vector<T>& v) {
    // Initialize original index locations.
    std::vector<int> idx(v.size(), 0);
    for (int k=0; k<idx.size(); k++) {
      idx[k] = k;
    }

    // Define a comparator lambda.
    auto comparator = [&v](int i1, int i2) -> bool {
      return (v[i1] < v[i2]);
    };

    // Sort indexes based on comparing values in v.
    std::sort(idx.begin(), idx.end(), comparator);

    return idx;
  }

  void sortSimplex() {
    // Get sorted indexes.
    std::vector<int> indexes = getSortedIndexes(fValues_);

    // Create local copies.
    Eigen::MatrixXd simplexCopy(simplex_);
    std::vector<double> fValuesCopy(fValues_);

    // Sort.
    for (size_t k=0; k<indexes.size(); k++) {
      simplexCopy.col(k) = simplex_.col(indexes[k]);
      fValuesCopy[k] = fValues_[indexes[k]];
    }

    // Update.
    simplex_ = simplexCopy;
    fValues_ = fValuesCopy;
  }

  void shrinkSimplex() {
    for (int k=1; k<numVertices_; k++) {
      simplex_.col(k) = simplex_.col(0) + sigma_*(simplex_.col(k)-simplex_.col(0));
      fValues_[k] = evalFcn(simplex_.col(k));
    }
  }

  void printSimplex() const {
    std::cout << "--------------------------------" << std::endl;
    std::cout << "Iteration: " << currentIteration_ << std::endl;
    std::cout << "Fcn values:";
    for (auto fValue : fValues_) {
      std::cout << " " << fValue;
    }
    std::cout << std::endl;
    std::cout << "Simplex:" << std::endl;
    std::cout << simplex_ << std::endl;
    std::cout << "--------------------------------" << std::endl << std::endl;
  }
};

} /* namespace optimization */

} /* namespace robot_utils */
