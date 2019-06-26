/*!
 * @file 	GNSS.hpp
 * @author 	Dominic Jud
 * @date 	March 28, 2017
 * @version 1.0
 * @ingroup robot_utils
 */

#pragma once

#include <Eigen/Dense>

namespace robot_utils {

class GNSS {
public:

    GNSS();
	virtual ~GNSS();

	void setReference(const double& referenceLatitude, const double& referenceLongitude, const double& referenceAltitude, const double& referenceHeading);
	Eigen::Vector3d gpsToCartesian(const double& latitudeInDegrees, const double& longitudeInDegrees, const double& altitude);
    Eigen::Vector3d cartesianToGps(const Eigen::Matrix<double,3,1> position);
    Eigen::Vector3d besselEllipsoidToMercator(const double& latitudeInRad, const double& longitudeInRad, const double& altitude);
    void setMercatorReferenceFrame(const Eigen::Vector3d newReferencePoint);

protected:
    void calculateConversionParameters();

    ///Reference longitude of the GPS measurements [deg]
    double referenceLongitude_;

    ///Reference latitude of the GPS measurements [deg]
    double referenceLatitude_;

    ///Reference altitude of the GPS measurements [m]
    double referenceAltitude_;

    ///Reference heading for the GPS measurements [rad]
    double referenceHeading_;

    //Conversion parameters
    double earthRadiusN_;
    double earthRadiusE_;

    //WGS84 constants for conversion
    const double EQUATORIAL_RADIUS = 6378137.0;
    const double FLATTENING = 1.0/298.257223563;
    //calculate WGS84 constants
    const double EXCENTRITY2 = 2*FLATTENING - FLATTENING*FLATTENING;

    // As desccribed in : https://www.swisstopo.admin.ch/content/swisstopo-internet/de/topics/survey/reference-systems/switzerland/_jcr_content/contentPar/tabs/items/dokumente_publikatio/tabPar/downloadlist/downloadItems/517_1459343190376.download/refsys_d.pdf
    // Major Axis Bessel Ellipsoid
    const double a = 6377397.155;
    // 1. numerical Excentricity Bessel Ellipsoid
    const double E2 = 0.006674372230614;
    // Latitude Bern
    const double phi0 = 46.95241*M_PI/180.0;
    // Longitude Bern
    const double lambda0 = 7.43958*M_PI/180.0;
    // Projection Sphere Radius
    const double R = a*sqrt(1-E2)/(1-E2*sin(phi0)*sin(phi0));
    // Ratio Sphere Length to Ellipsoid Length;
    const double alpha = sqrt(1+E2/(1-E2)*pow(cos(phi0),4));
    // Latitude of Zero Point on Sphere
    const double b0 = asin(sin(phi0)/alpha);
    // Konstante der Breitenformel
    const double K = log(tan(M_PI_4+b0/2.0))-alpha*log(tan(M_PI_4+phi0/2.0))+alpha*sqrt(E2)/2.0*log((1+sqrt(E2)*sin(phi0))/(1-sqrt(E2)*sin(phi0)));
    // xyz-offset
    Eigen::Vector3d xyzOffset = Eigen::Vector3d::Zero();
};

} /* namespace robot_utils */
