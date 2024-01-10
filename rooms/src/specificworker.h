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
#include <ctime>
#include <chrono>
#include "graph.h"

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
    const float LOW_LOW= 0;
    const float LOW_HIGH= 400;
    const float MIDDLE_LOW= 800;
    const float MIDDLE_HIGH= 1200;
    const float HIGH_LOW= 1600;
    const float HIGH_HIGH= 2000;

    bool startup_check_flag;
    AbstractGraphicViewer* viewer;

    struct Lines{
        RoboCompLidar3D::TPoints low,middle,high;
    };
    struct Door{
        struct Point{
            double x,y;
        };
        RoboCompLidar3D::TPoint right,left,middle;
        Door(const RoboCompLidar3D::TPoint &right_, const RoboCompLidar3D::TPoint &left_) : right(right_),left(left_)
        {
            middle.x = (left.x + right.x)/2;
            middle.y = (left.y + right.y)/2;
        }
        Door(){right = left = middle = RoboCompLidar3D::TPoint(0,0,0);};
        bool operator==(const Door &d)const
        {
            return std::hypot(d.middle.x - this->middle.x, d.middle.y - this->middle.y) < 500;
        }
        Door& operator=(const Door &d){
            left=d.left;
            right=d.right;
            middle=d.middle;
            return *this;
        };
        //Calcula la distancia del robot con respecto a la puerta
        float dist_to_robot() const
        { return std::hypot(middle.x,middle.y); }
        //Calcula el angulo del robot con respecto a la puerta
        float angle_to_robot() const
        { return atan2(middle.x,middle.y);}
        Point perp_point() const
        {
            //Calcula un vector perpendicular a la puerta
            Point d_perp;
            d_perp.x = -(left.y - right.y);
            d_perp.y = left.x - right.x;

            //Transforma el vector a un vector unitario
            double magnitude = std::sqrt(std::pow(d_perp.x, 2) + std::pow(d_perp.y, 2));
            Point u_perp;
            u_perp.x = d_perp.x / magnitude;
            u_perp.y = d_perp.y / magnitude;

            //Calcula los dos puntos perpendiculares a la puerta aplicando el vector
            // al punto medio de la puerta y se queda con el mas cercano
            Point a,b;
            Point M{middle.x, middle.y};
            a.x = M.x + u_perp.x * 1000;
            a.y = M.y + u_perp.y * 1000;
            b.x = M.x - u_perp.x * 1000;
            b.y = M.y - u_perp.y * 1000;
            float len_a = std::hypot(a.x, a.y);
            float len_b = std::hypot(b.x, b.y);
            return len_a < len_b ? a : b;
        }
        //Calcula la distancia del robot con respecto al punto perpendicular de la puerta
        float perp_dist_to_robot() const
        {
            auto p = perp_point();
            return std::hypot(p.x, p.y);
        }
        //Calcula el angulo del robot con respecto al punto perpendicular de la puerta
        float perp_angle_to_robot() const
        {
            auto p = perp_point();
            return atan2(p.x, p.y);
        }
    };
    using Doors = std::vector<Door>;

    enum class Modo { IDLE, CROSS_DOOR, MOVE_TO_DOOR, SELECT_DOOR};
    Modo modo = Modo::SELECT_DOOR;
    Door door_target;
    int cont_h=0;
    int hab_actual;
    Graph graph;
    float v_rot,v_adv,v_lat;
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    SpecificWorker::Lines get_lines(RoboCompLidar3D::TPoints &points);
    SpecificWorker::Lines extract_peaks(const SpecificWorker::Lines &lines);
    std::tuple<SpecificWorker::Doors,SpecificWorker::Doors,SpecificWorker::Doors> get_doors(const SpecificWorker::Lines &peaks);
    SpecificWorker::Doors get_true_doors(const std::tuple<SpecificWorker::Doors,SpecificWorker::Doors,SpecificWorker::Doors> &doors);
    SpecificWorker::Door select_door(Doors &true_doors);
    float break_adv(float dist_to_target);
    float break_rot(float rot);
    std::tuple<SpecificWorker::Modo, float, float, float> moveToDoor(SpecificWorker::Door d_target);
    std::tuple<SpecificWorker::Modo, float, float, float> crossDoor(SpecificWorker::Door d_target);
    void draw_lidar(RoboCompLidar3D::TPoints &points,AbstractGraphicViewer *viewer);
    void draw_peaks(const Lines &peaks, AbstractGraphicViewer *viewer);
    void draw_doors(const Doors &doors, AbstractGraphicViewer *viewer);
};

#endif