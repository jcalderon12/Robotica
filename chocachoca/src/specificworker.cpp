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
        std::tuple<SpecificWorker::Modo, float, float, float> state = make_tuple(modo,v_adv,v_lat,v_rot);

        switch (modo) {
            case Modo::IDLE:

                break;
            case Modo::FOLLOW_WALL:
                state = follow_wall(filtered_points, state);
                break;
            case Modo::SPIRAL:
                state= spiral(filtered_points, state);
                break;
            case Modo::TURN:
                state = turn(filtered_points, state);
                break;
            case Modo::STRAIGHT_LINE:
                state = straight_line(filtered_points, state);
                break;
            case Modo::CHOCACHOCA:
                chocachoca(filtered_points);
                break;
        }
        modo = std::get<0>(state);
        v_adv = std::get<1>(state);
        v_lat= std::get<2>(state);
        v_rot = std::get<3>(state);
        qInfo() << "v rot:" << v_rot << "v_lat:" << v_lat << "v adv:" << v_adv;
        omnirobot_proxy->setSpeedBase(v_adv/1000.f,v_lat/1000.f,v_rot);
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

std::tuple<SpecificWorker::Modo, float , float, float> SpecificWorker::straight_line(RoboCompLidar3D::TPoints &filtered_points, std::tuple<SpecificWorker::Modo, float, float, float> state)
{
    SpecificWorker::Modo _modo=std::get<0>(state);
    float _v_adv=std::get<1>(state);
    float _v_lat=std::get<2>(state);
    float _v_rot=std::get<3>(state);
    int offset = filtered_points.size()/2-filtered_points.size()/18;

    auto min_elem = std::min_element(filtered_points.begin()+offset,filtered_points.end()-offset,
                                     [](auto a, auto b) {return std::hypot(a.x,+a.y) < std::hypot(b.x,b.y); });
    const float MIN_DISTANCE = 800;
    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {
        qInfo() << __FUNCTION__ << "collision";
        return make_tuple(Modo::TURN,0,0,3);
    }
    //Detectar si hay mas de 5000 de distancia con cualquier punto para pasar a spiral
    min_elem = std::min_element(filtered_points.begin(),filtered_points.end(),
                                [](auto a, auto b) {return std::hypot(a.x,+a.y) < std::hypot(b.x,b.y); });
    if(std::hypot(min_elem->x,min_elem->y) > 2000)
        return make_tuple(Modo::SPIRAL,500,0,3);
    else
    {
        _v_adv=2000;
        _v_rot=0;
    }
    return make_tuple(_modo,_v_adv,_v_lat,_v_rot);
}

std::tuple<SpecificWorker::Modo, float, float, float> SpecificWorker::turn(RoboCompLidar3D::TPoints &filtered_points, std::tuple<SpecificWorker::Modo, float, float, float> state)
{
    SpecificWorker::Modo _modo=std::get<0>(state);
    float _v_adv=std::get<1>(state);
    float _v_lat=std::get<2>(state);
    float _v_rot=std::get<3>(state);

    int offset = filtered_points.size()/2-filtered_points.size()/18;

    srand(time(0));

    auto min_elem = std::min_element(filtered_points.begin()+offset,filtered_points.end()-offset,
                                     [](auto a, auto b) {return std::hypot(a.x,+a.y) < std::hypot(b.x,b.y); });
    const float MIN_DISTANCE = 600;
    if(std::hypot(min_elem->x, min_elem->y) > MIN_DISTANCE)
    {
        int random = rand() % 3;
        if(random == 0)
            return make_tuple(Modo::STRAIGHT_LINE,2000,0,0);
        else
        {
            n_fw++;
            return make_tuple(Modo::FOLLOW_WALL,2000,0,0);
        }

    }
    return make_tuple(_modo,_v_adv,_v_lat,_v_rot);
}

std::tuple<SpecificWorker::Modo, float, float, float> SpecificWorker::follow_wall(RoboCompLidar3D::TPoints &filtered_points, std::tuple<SpecificWorker::Modo, float, float,float> state)
{
    SpecificWorker::Modo _modo=std::get<0>(state);
    float _v_adv=std::get<1>(state);
    float _v_lat=std::get<2>(state);
    float _v_rot=std::get<3>(state);

    int offset = filtered_points.size()/2-filtered_points.size()/18;
    srand(time(0));

    auto min_elem = std::min_element(filtered_points.begin()+offset,filtered_points.end()-offset,
                                     [](auto a, auto b) {return std::hypot(a.x,+a.y) < std::hypot(b.x,b.y); });
    const float MIN_DISTANCE = 600;
    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE)
    {
            return make_tuple(Modo::TURN,0,0,3);
    }
    offset = filtered_points.size()*3/4-filtered_points.size()/8;
    int offset2 = filtered_points.size()*3/4+filtered_points.size()/8;
    min_elem = std::min_element(filtered_points.begin()+offset,filtered_points.begin()+offset2,
                                     [](auto a, auto b) {return std::hypot(a.x,+a.y) < std::hypot(b.x,b.y); });
    const float REF_DISTANCE = 600+(200*(n_fw/4));
    if(std::hypot(min_elem->x, min_elem->y) < REF_DISTANCE - 100)
    {
        _v_adv=600;
        _v_lat=1000;
    }
    else {
        if (std::hypot(min_elem->x, min_elem->y) > REF_DISTANCE + 100) {
            _v_adv = 600;
            _v_lat = -1000;
        }
    }
    return make_tuple(_modo,_v_adv,_v_lat,_v_rot);
}

std::tuple<SpecificWorker::Modo, float, float, float> SpecificWorker::spiral(RoboCompLidar3D::TPoints &filtered_points, std::tuple<SpecificWorker::Modo, float, float, float> state)
{
    SpecificWorker::Modo _modo=std::get<0>(state);
    float _v_adv=std::get<1>(state);
    float _v_lat=std::get<2>(state);
    float _v_rot=std::get<3>(state);

    int offset = filtered_points.size()/2-filtered_points.size()/18  ;

    auto min_elem = std::min_element(filtered_points.begin()+offset,filtered_points.end()-offset,
                                     [](auto a, auto b) {return std::hypot(a.x,+a.y) < std::hypot(b.x,b.y); });
    const float MIN_DISTANCE = 800;
    if(std::hypot(min_elem->x, min_elem->y) > MIN_DISTANCE)
        return make_tuple(Modo::SPIRAL,_v_adv+7,0,_v_rot-0.009);
    else
        return make_tuple(Modo::TURN,0,0,2);
}

void SpecificWorker::chocachoca(RoboCompLidar3D::TPoints &filtered_points)
{
    int offset = filtered_points.size()/2-filtered_points.size()/18  ;

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
        //Detectar si hay mas de 5000 de distancia con cualquier punto para pasar a spiral
        min_elem = std::min_element(filtered_points.begin(),filtered_points.end(),
                                    [](auto a, auto b) {return std::hypot(a.x,+a.y) < std::hypot(b.x,b.y); });
        if(std::hypot(min_elem->x,min_elem->y) > 2000)
            modo = Modo::SPIRAL;
    }
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




/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

