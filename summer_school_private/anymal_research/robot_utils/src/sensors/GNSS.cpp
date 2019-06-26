/*!
 * @file 	GNSS.cpp
 * @author 	Dominic Jud
 * @date 	March 28, 2017
 * @version 1.0
 * @ingroup robot_utils
 */

#include "robot_utils/sensors/GNSS.hpp"

#include <stdio.h>
#include <iostream>
#include <iomanip>

namespace robot_utils {


GNSS::GNSS() :
    referenceLongitude_(7.43958),
    referenceLatitude_(46.95241),
    referenceAltitude_(0.0),
    referenceHeading_(0.0)
{
    calculateConversionParameters();
}

GNSS::~GNSS()
{
}

void GNSS::calculateConversionParameters(){
    // calculate earth radii
    const double temp = 1.0 / (1.0 - EXCENTRITY2 * sin(referenceLatitude_ * M_PI/180.0) * sin(referenceLatitude_ * M_PI/180.0));
    const double prime_vertical_radius = EQUATORIAL_RADIUS * sqrt(temp);
    earthRadiusN_ = prime_vertical_radius * (1 - EXCENTRITY2) * temp;
    earthRadiusE_  = prime_vertical_radius * cos(referenceLatitude_ * M_PI/180.0);
}

void GNSS::setReference(const double& referenceLatitude, const double& referenceLongitude, const double& referenceAltitude, const double& referenceHeading){
    referenceLatitude_ = referenceLatitude;
    referenceLongitude_ = referenceLongitude;
    referenceAltitude_ = referenceAltitude;
    referenceHeading_ = referenceHeading;

    calculateConversionParameters();
}

Eigen::Vector3d GNSS::gpsToCartesian(const double& latitudeInDegrees, const double& longitudeInDegrees, const double& altitude){
    const double cn = cos(referenceHeading_);
    const double sn = sin(referenceHeading_);
    const double kn = 180.0/earthRadiusN_/M_PI;
    const double ke = 180.0/earthRadiusE_/M_PI;
    const double lat_tmp = (latitudeInDegrees - referenceLatitude_)/kn;
    const double lon_tmp = (longitudeInDegrees - referenceLongitude_)/ke;

    Eigen::Vector3d position;
    position(0) = cn*lat_tmp + sn*lon_tmp;
    position(1) = sn*lat_tmp - cn*lon_tmp;
    position(2) = altitude - referenceAltitude_;

    return position;
}

Eigen::Vector3d GNSS::cartesianToGps(const Eigen::Matrix<double,3,1> position) {
    Eigen::Vector3d gpsCoordinates;
    gpsCoordinates(0) = referenceLatitude_ +
                    (cos(referenceHeading_) * position(0) + sin(referenceHeading_) * position(1)) / earthRadiusN_ *
                    180.0 / M_PI;
    gpsCoordinates(1) = referenceLongitude_ -
                     (-sin(referenceHeading_) * position(0) + cos(referenceHeading_) * position(1)) / earthRadiusE_ *
                     180.0 / M_PI;
    gpsCoordinates(2) = referenceAltitude_ + position(2);

    return gpsCoordinates;
}

void GNSS::setMercatorReferenceFrame(const Eigen::Vector3d newReferencePoint) {
    xyzOffset = newReferencePoint;
}

Eigen::Vector3d GNSS::besselEllipsoidToMercator(const double& latitudeInRad, const double& longitudeInRad, const double& altitude){
    //Ellipsoid to Sphere (Gauss)
    const double S = alpha*log(tan(M_PI_4+latitudeInRad/2.0)) - alpha*sqrt(E2)/2.0*log((1+sqrt(E2)*sin(latitudeInRad))/(1-sqrt(E2)*sin(latitudeInRad))) + K;
    const double b = 2*(atan(exp(S))-M_PI_4);
    const double l = alpha*(longitudeInRad-lambda0);

    //Equator to Pseudoequator (Rotation)
    const double lHat = atan(sin(l)/(sin(b0)*tan(b)+cos(b0)*cos(l)));
    const double bHat = asin(cos(b0)*sin(b)-sin(b0)*cos(b)*cos(l));

    //Sphere to Plane (Mercator)
    const double Y = R*lHat;
    const double X = R/2.0*log((1+sin(bHat))/(1-sin(bHat)));

    return Eigen::Vector3d(Y,X,altitude)-xyzOffset; //Yes, this is correct. A point in swiss coordinates is denoted as (y,x). See https://en.wikipedia.org/wiki/Swiss_coordinate_system
}

} /* namespace robot_utils */
