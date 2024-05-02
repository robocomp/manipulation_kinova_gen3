/*
 *    Copyright (C) 2024 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{

        bool success = true;
        std::string robot_name = "my_gen3";
        //-------------------------------------------------------------
        // Get the intrinsic parameters for a given sensor
        success &= example_get_intrinsic_parameters( robot_name);
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	
	
}

bool SpecificWorker::example_get_intrinsic_parameters( const std::string& robot_name)
{
    //ros::ServiceClient service_client_get_intrinsic_parameters = n.serviceClient<kortex_driver::GetIntrinsicParameters>("/" + robot_name + "/vision_config/get_intrinsic_parameters");
    //kortex_driver::GetIntrinsicParameters service_get_intrinsic_parameters;

    // Get the parameters from the color sensor
    kortex_driver::GetIntrinsicParametersRequest req;
    req.input.sensor = kortex_driver::Sensor::SENSOR_COLOR; // Change to SENSOR_DEPTH for the depth sensor
    //service_get_intrinsic_parameters.request = req;

    /*if (!service_client_get_intrinsic_parameters.call(service_get_intrinsic_parameters))
    {
        std::string error_string = "Failed to call GetIntrinsicParameters";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    auto output = service_get_intrinsic_parameters.response.output;
    */
    // The message description can be seen at msg/generated/vision_config/IntrinsicParameters.msg
    std::ostringstream oss;
    oss << std::endl << "---------------------------------" << std::endl
        << "Intrinsic parameters are : " << std::endl
        << "Focal length in x is : " << output.focal_length_x << std::endl
        << "Focal length in y is : " << output.focal_length_y << std::endl
        << "Principal point in x is : " << output.principal_point_x << std::endl
        << "Principal point in y is : " << output.principal_point_y << std::endl
        << "Distortion coefficients are : [" <<
        "k1 = " << output.distortion_coeffs.k1 << "; " <<
        "k2 = " << output.distortion_coeffs.k2 << "; " <<
        "k3 = " << output.distortion_coeffs.k3 << "; " <<
        "p1 = " << output.distortion_coeffs.p1 << "; " <<
        "p2 = " << output.distortion_coeffs.p2 << "]" << std::endl;

    // The Sensor enum can be seen at msg/generated/vision_config/Sensor.msg
    oss << "Sensor type is : " << sensor_type_enum_to_string(output.sensor) << std::endl;

    // The Resolution enum can be seen at msg/generated/vision_config/Resolution.msg
    oss << "Resolution is : " << resolution_enum_to_string(output.resolution) << std::endl
        << "---------------------------------";

    ROS_INFO("%s", oss.str().c_str());

    return true;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
//implementCODE

}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
//implementCODE

}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
//implementCODE

}

RoboCompCameraRGBDSimple::TPoints SpecificWorker::CameraRGBDSimple_getPoints(std::string camera)
{
//implementCODE

}



/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::Point3D
// RoboCompCameraRGBDSimple::TPoints
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

