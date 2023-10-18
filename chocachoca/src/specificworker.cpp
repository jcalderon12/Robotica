/*
 *    Copyright (C) 2023 by YOUR NAME HERE
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
        // Inicializaciones personales
        viewer = new AbstractGraphicViewer(this,QRectF(-5000,-5000,10000,10000));
        viewer->add_robot(460,480,0,10,QColor("Blue"));
        viewer->show();
        viewer->activateWindow();
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
	try
	{
        auto ldata =lidar3d_proxy->getLidarData("bpearl", 0, 360, 1);
        qInfo() << ldata.points.size();
        const auto &points = ldata.points;
        if( points.empty()) return;

        RoboCompLidar3D::TPoints filtered_points;
        std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p){ return p.z < 2000;});
        draw_lidar(filtered_points,viewer);

        /// control

        switch (modo) {
            case Modo::IDLE:
                    omnirobot_proxy->setSpeedBase(0,0,0);
                break;
            case Modo::FOLLOW_WALL:
                follow_wall(filtered_points);
                break;
            case Modo::SPIRAL:

                break;
            case Modo::STRAIGHT_LINE:
                straight_line(filtered_points);
                break;
            case Modo::CHOCACHOCA:
                chocachoca(filtered_points);
                break;
            default:
                omnirobot_proxy->setSpeedBase(0,0,0);
        }
    }
	catch(const Ice::Exception &e)
	{
	  std::cout << "Error reading from Camera" << e << std::endl;
	}
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::follow_wall(RoboCompLidar3D::TPoints &f_points)
{
    int offset1 = f_points.size()/2;
    int offset2 = f_points.size()*3/4;
    if(hypot(f_points[offset1].x,f_points[offset1].y) < 900) //CALCULAR PARA QUE EL ANGULO DEL ROBOT CON LA PARED SEA DE 90º
        omnirobot_proxy->setSpeedBase(0, 0, 0.5);
    else
    {
        if(hypot(f_points[offset2].x,f_points[offset2].y) < 300)
            omnirobot_proxy->setSpeedBase(0, 0, 0.5);
        else
            omnirobot_proxy->setSpeedBase(1000/1000.f, 0, 0);
    }
}

void SpecificWorker::spiral(RoboCompLidar3D::TPoints &f_points)
{
    float acum = 0.0;
    omnirobot_proxy->setSpeedBase(100/1000.f, 0, 0.1*acum);

}

void SpecificWorker::straight_line(RoboCompLidar3D::TPoints &f_points)
{
    int offset = f_points.size()/2;
    if (hypot(f_points[offset].x,f_points[offset].y) > 1000)
        omnirobot_proxy->setSpeedBase(1000/1000.f, 0, 0);
    else
        modo = Modo::IDLE;
}

void SpecificWorker::chocachoca(RoboCompLidar3D::TPoints &filtered_points)
{
    int offset = filtered_points.size()/2-filtered_points.size()/5;

    auto min_elem = std::min_element(filtered_points.begin()+offset,filtered_points.end()-offset,
                                     [](auto a, auto b) {return std::hypot(a.x,+a.y) < std::hypot(b.x,b.y); });
    //qInfo() << min_elem->x << min_elem->y << min_elem->z;
    const float MIN_DISTANCE = 800;
    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {
        // STOP the robot && START
        try
        {
            omnirobot_proxy->setSpeedBase(0,0,0.5);
        }
        catch (const Ice::Exception &e)
        { std::cout << "Error reading from Camera" << e << std::endl;
        }
    }
    else
    {
        //start the robot
        try
        {
            omnirobot_proxy->setSpeedBase(1000/1000.f, 0, 0);
        }
        catch (const Ice::Exception &e)
        { std::cout << "Error reading from Camera" << e << std::endl;
        }
    }
}

void SpecificWorker::draw_lidar(RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
    //Obtener campo de vision
    int offset = points.size()/2-points.size()/5;
    auto p1 = points.at(offset);
    float alpha = atan2(p1.x,p1.y);

    static std::vector<QGraphicsItem*> borrar;
    static QGraphicsItem *borrarl1;
    static QGraphicsItem *borrarl2;
    for(auto &b: borrar)
    {
        viewer->scene.removeItem(b);
        delete b;
    }
    if(borrarl1 != nullptr)
    {
        viewer->scene.removeItem(borrarl1);
        delete borrarl1;
    }
    if(borrarl2 != nullptr)
    {
        viewer->scene.removeItem(borrarl2);
        delete borrarl2;
    }

    borrar.clear();

    borrarl1 = viewer->scene.addLine(0,0,5000*sin(alpha), 5000*cos(alpha),QPen(QColor("red"),50));
    borrarl2 = viewer->scene.addLine(0,0,5000*sin(-alpha), 5000*cos(-alpha),QPen(QColor("green"),50));
    for(const auto &p: points)
    {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen("blue"),QBrush(QColor("blue")));
        point->setPos(p.x,p.y);
        borrar.push_back(point);
    }
}




/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

