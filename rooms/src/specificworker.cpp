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
#include "cppitertools/sliding_window.hpp"
#include "cppitertools/combinations.hpp"

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
        auto ldata =lidar3d_proxy->getLidarData("helios", 0, 360, 1);
        qInfo() << ldata.points.size();
        const auto &points = ldata.points;
        if( points.empty()) return;

        RoboCompLidar3D::TPoints filtered_points;
        std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p){ return p.z < 1200;});
        draw_lidar(filtered_points,viewer);

        struct Lines lineas = obtener_lineas(filtered_points);
        lineas = extraer_picos(lineas);
        Doors doors = get_doors(lineas);
        draw_doors(doors , viewer);

        /// control
    /*    std::tuple<SpecificWorker::Modo, float, float, float> state = make_tuple(modo,v_adv,v_lat,v_rot);

        switch (modo) {
            case Modo::IDLE:

                break;
        }
        modo = std::get<0>(state);
        v_adv = std::get<1>(state);
        v_lat= std::get<2>(state);
        v_rot = std::get<3>(state);
        qInfo() << "v rot:" << v_rot << "v_lat:" << v_lat << "v adv:" << v_adv;
        omnirobot_proxy->setSpeedBase(v_adv/1000.f,v_lat/1000.f,v_rot);*/
    }
	catch(const Ice::Exception &e)
	{
	  std::cout << "Error reading from Camera" << e << std::endl;
	}
}

SpecificWorker::Lines SpecificWorker::obtener_lineas(RoboCompLidar3D::TPoints &points){
    const float bajo_bajo= 200;
    const float bajo_alto= 400;
    const float medio_bajo= 600;
    const float medio_alto= 800;
    const float alto_bajo= 1000;
    const float alto_alto= 1200;
    struct Lines lineas;
    for (auto p : points){
        if (p.z > bajo_bajo and p.z < bajo_alto ) lineas.bajo.push_back(p);
        else {
            if (p.z > medio_bajo and p.z < medio_alto) lineas.medio.push_back(p);
            else if (p.z > alto_bajo and  p.z < alto_alto ) lineas.alto.push_back(p);
        }
    }
    return lineas;
}

SpecificWorker::Lines SpecificWorker::extraer_picos(const SpecificWorker::Lines &lines)
{
    Lines peaks;
    const float THRESH = 500;

    for(const auto &both : iter::sliding_window(lines.medio, 2))
        if(fabs(both [1].r - both[0].r) > THRESH)
            peaks.bajo.push_back(both[0]);

    return peaks;
}

SpecificWorker::Doors SpecificWorker::get_doors(const SpecificWorker::Lines &peaks){
    auto dist = [](auto a, auto b)
            { return std::hypot(a.x-b.x, a.y-b.y); };

    Doors doors;
    const float THRES_DOOR = 100;

    auto near_door = [doors, dist, THRES_DOOR](auto d)
            { for(auto &&old : doors)
                {
                    if(dist(old.left, d.left) < THRES_DOOR or
                       dist(old.right, d.right) < THRES_DOOR or
                       dist(old.left, d.right) < THRES_DOOR or
                       dist(old.right, d.left) < THRES_DOOR)
                       return true;
                    else
                        return false;
                }
            };
    for(auto &&par : peaks.medio | iter::combinations(2))
    {
        if(dist(par[0], par[1]) < 1300 and dist(par[0],par[1]) > 500)
        {
            auto door = Door{par[0],par[1]};
            if(not near_door(door))
                doors.emplace_back(door);
        }
    }
    return doors;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::draw_lidar(RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
    //Obtener campo de vision
    int offset = points.size()/2-points.size()/18;
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

void SpecificWorker::draw_doors(const Doors &doors, AbstractGraphicViewer *viewer){

    static std::vector<QGraphicsItem*> borrar;
    for(auto &b: borrar)
    {
        viewer->scene.removeItem(b);
        delete b;
    }

    borrar.clear();

    for(const auto &d: doors)
    {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen("green"),QBrush(QColor("green")));
        point->setPos(d.left.x, d.left.y);
        borrar.push_back(point);
        point = viewer->scene.addRect(-50, -50, 100, 100, QPen("green"),QBrush(QColor("green")));
        point->setPos(d.right.x, d.right.y);
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

