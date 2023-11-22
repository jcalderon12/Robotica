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
        //qInfo() << ldata.points.size();
        const auto &points = ldata.points;
        if( points.empty()) return;

        RoboCompLidar3D::TPoints filtered_points;
        std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p){ return p.z < 2000 and p.z > 100;});

        auto lines = get_lines(filtered_points);
        auto peaks = extract_peaks(lines);
        auto doors = get_doors(peaks);
        auto true_doors = get_true_doors(doors);

        draw_lidar(filtered_points,viewer);
        draw_peaks(peaks, viewer);
        draw_doors(true_doors , viewer);
        for(auto d : true_doors)
            qInfo() << d.right.r << d.left.r ;

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

SpecificWorker::Lines SpecificWorker::get_lines(RoboCompLidar3D::TPoints &points)
{
    struct Lines lines;
    for (auto p : points){
        if (p.z > LOW_LOW and p.z < LOW_HIGH ) lines.low.push_back(p);
        else {
            if (p.z > MIDDLE_LOW and p.z < MIDDLE_HIGH) lines.middle.push_back(p);
            else if (p.z > HIGH_LOW and  p.z < HIGH_HIGH ) lines.high.push_back(p);
        }
    }
    return lines;
}

SpecificWorker::Lines SpecificWorker::extract_peaks(const SpecificWorker::Lines &lines)
{
    Lines peaks;
    const float THRESH = 600;

    for(const auto &both : iter::sliding_window(lines.low, 2))
        if(fabs(both [1].r - both[0].r) > THRESH)
            peaks.low.push_back(both[0].r < both[1].r ? both[0] : both[1]);
    for(const auto &both : iter::sliding_window(lines.middle, 2))
        if(fabs(both [1].r - both[0].r) > THRESH)
            peaks.middle.push_back(both[0].r < both[1].r ? both[0] : both[1]);
    for(const auto &both : iter::sliding_window(lines.high, 2))
        if(fabs(both [1].r - both[0].r) > THRESH)
            peaks.high.push_back(both[0].r < both[1].r ? both[0] : both[1]);

    return peaks;
}

std::tuple<SpecificWorker::Doors,SpecificWorker::Doors,SpecificWorker::Doors> SpecificWorker::get_doors(const Lines &peaks){
    auto dist = [](auto a, auto b)
            { return std::hypot(a.x-b.x, a.y-b.y); };

    Doors doors_low,doors_middle,doors_high;
    const float THRESH_DOOR = 200;

    auto near_door = [dist, THRESH_DOOR](auto d, Doors &doors)
            { for(auto &&old : doors)
                {
                    if(dist(old.left, d.left) < THRESH_DOOR or
                       dist(old.right, d.right) < THRESH_DOOR or
                       dist(old.left, d.right) < THRESH_DOOR or
                       dist(old.right, d.left) < THRESH_DOOR)
                       return true;
                }
                return false;
            };

    for(auto &&par : peaks.low | iter::combinations(2))
    {
        if(dist(par[0], par[1]) < 1400 and dist(par[0],par[1]) > 500)
        {
            auto door = Door{par[0],par[1]};
            if(not near_door(door,doors_low))
                doors_low.emplace_back(door);
        }
    }

    for(auto &&par : peaks.middle | iter::combinations(2))
    {
        if(dist(par[0], par[1]) < 1400 and dist(par[0],par[1]) > 500)
        {
            auto door = Door{par[0],par[1]};
            if(not near_door(door,doors_middle))
                doors_middle.emplace_back(door);
        }
    }

    for(auto &&par : peaks.high | iter::combinations(2))
    {
        if(dist(par[0], par[1]) < 1400 and dist(par[0],par[1]) > 500)
        {
            auto door = Door{par[0],par[1]};
            if(not near_door(door,doors_high))
                doors_high.emplace_back(door);
        }
    }
    std::tuple<Doors,Doors,Doors> doors = make_tuple(doors_low,doors_middle,doors_high);
    return doors;
}

SpecificWorker::Doors SpecificWorker::get_true_doors(const std::tuple<Doors,Doors,Doors> &doors)
{
    Doors doors_low = get<0>(doors);
    Doors doors_middle = get<1>(doors);
    Doors doors_high = get<2>(doors);
    Doors true_doors;
    for(auto d : doors_low) {
        bool high_check = false;
        bool middle_check = false;
        int i = 0;
        int j = 0;
        while (i < doors_middle.size() && !middle_check) {
            if (doors_middle[i] == d) middle_check = true;
            i++;
        }
        while(j<doors_high.size() && !high_check) {
            if (doors_high[j] == d) high_check = true;
            j++;
        }
        if(high_check && middle_check)
            true_doors.emplace_back(d);
    }
    return true_doors;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::draw_lidar(RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b: borrar)
    {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for(const auto &p: points)
    {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen("blue"),QBrush(QColor("blue")));
        point->setPos(p.x,p.y);
        borrar.push_back(point);
    }
}

void SpecificWorker::draw_peaks(const Lines &peaks, AbstractGraphicViewer *viewer){

    static std::vector<QGraphicsItem*> borrar;
    for(auto &b: borrar)
    {
        viewer->scene.removeItem(b);
        delete b;
    }

    borrar.clear();

    for(const auto &d: peaks.low)
    {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen("yellow"),QBrush(QColor("yellow")));
        point->setPos(d.x, d.y);
        borrar.push_back(point);
    }

    for(const auto &d: peaks.middle)
    {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen("black"),QBrush(QColor("black")));
        point->setPos(d.x, d.y);
        borrar.push_back(point);
    }

    for(const auto &d: peaks.high)
    {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen("red"),QBrush(QColor("red")));
        point->setPos(d.x, d.y);
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
        auto line = viewer->scene.addLine(d.left.x,d.left.y,d.right.x,d.right.y,QPen(QColor("green"),50));
        borrar.push_back(line);
    }
}


/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

