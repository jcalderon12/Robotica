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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <ranges>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
	bool startup_check_flag;
    AbstractGraphicViewer* viewer;

    enum class Modo { IDLE, FOLLOW_WALL, STRAIGHT_LINE, TURN, SPIRAL, CHOCACHOCA };
    Modo modo = Modo::STRAIGHT_LINE; //CAMBIAR A FOLLOW_WALL PARA MODO DIENTES DE SIERRA
    float v_rot,v_adv,v_lat;
    //int n_fw=0; //NECESARIO PARA MODO DIENTES DE SIERRA
    std::tuple<SpecificWorker::Modo, float , float, float> straight_line(RoboCompLidar3D::TPoints &f_points, std::tuple<SpecificWorker::Modo, float, float, float> state);
    std::tuple<SpecificWorker::Modo, float , float, float> turn(RoboCompLidar3D::TPoints &f_points, std::tuple<SpecificWorker::Modo, float, float, float> state);
    std::tuple<SpecificWorker::Modo, float, float, float> follow_wall(RoboCompLidar3D::TPoints &f_points, std::tuple<SpecificWorker::Modo, float, float, float> state);
    std::tuple<SpecificWorker::Modo, float, float, float> spiral(RoboCompLidar3D::TPoints &f_points, std::tuple<SpecificWorker::Modo, float, float, float> state);
    void chocachoca(RoboCompLidar3D::TPoints &f_points);
    void draw_lidar(RoboCompLidar3D::TPoints &points,AbstractGraphicViewer *viewer);
};

#endif
