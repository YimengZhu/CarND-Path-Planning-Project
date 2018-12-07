#include "spline.h"
#include <vector>
#include <math.h>
#include "Planner.hpp"
#include <iostream>

using namespace std;

namespace planning{

        Planner::Planner(double car_x, double car_y, double car_yaw, vector<double> previous_path_x, vector<double> previous_path_y)
        :m_startYaw(deg2rad(car_yaw)),
        m_planSize(50 - previous_path_x.size())
        {            
            if (previous_path_x.size() < 2) 
            {
                m_startX = car_x;
                m_startY = car_y;

                double prev_x = car_x - cos(car_yaw);
                double prev_y = car_y - sin(car_yaw);

                m_ptsx.push_back(prev_x);
                m_ptsx.push_back(car_x);

                m_ptsy.push_back(prev_y);
                m_ptsy.push_back(car_y);
            }
            else 
            {
                m_startX = previous_path_x[previous_path_x.size() - 1];
                m_startY = previous_path_y[previous_path_y.size() - 1];

                double x_prev = previous_path_x[previous_path_x.size() - 2];
                double y_prev = previous_path_y[previous_path_y.size() - 2];
                m_startYaw = atan2(m_startY - y_prev, m_startX - x_prev);

                m_ptsx.push_back(x_prev);
                m_ptsx.push_back(m_startX);

                m_ptsy.push_back(y_prev);
                m_ptsy.push_back(m_startY);
            }


        }

        Planner::~Planner(){

        }

        void Planner::calculateSplineWithWayPoints(vector<double> waypoint1, vector<double> waypoint2, vector<double> waypoint3){
            m_ptsx.push_back(waypoint1[0]);
            m_ptsx.push_back(waypoint2[0]);
            m_ptsx.push_back(waypoint3[0]);

            m_ptsy.push_back(waypoint1[1]);
            m_ptsy.push_back(waypoint2[1]);
            m_ptsy.push_back(waypoint3[1]);

            for(int  i = 0 ; i < m_ptsx.size(); i++){
                cout<<"x: "<<m_ptsx[i]<<", y: "<<m_ptsy[i]<<endl;
            }
            shiftToStartPoint();
            for(int  i = 0 ; i < m_ptsx.size(); i++){
                cout<<"new x: "<<m_ptsx[i]<<", new y: "<<m_ptsy[i]<<endl;
            }
            m_spline.set_points(m_ptsx, m_ptsy);
        }
        

        void Planner::plan(vector<double> &next_x_vals, vector<double> &next_y_vals, double speed) {
            double target_x = 30;
            double target_y = m_spline(target_x);
            double target_dist = sqrt(target_x * target_x + target_y * target_y);

            double x_shift_buffer = 0;
            for (int i = 1; i < m_planSize; i++) 
            {
                double N = target_dist / (0.02 * speed / 2.24);
                double x_point = x_shift_buffer + target_x / N;
                double y_point = m_spline(x_point);

                x_shift_buffer = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                x_point = x_ref * cos(m_startYaw) - y_ref * sin(m_startYaw);
                y_point = x_ref * sin(m_startYaw) + y_ref * cos(m_startYaw);

                x_point += m_startX;
                y_point += m_startY;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }  
        }

        
            double Planner::deg2rad(double x) { return x * M_PI / 180; }
            double Planner::rad2deg(double x) { return x * 180 / M_PI; }

            void Planner::shiftToStartPoint(){
                for (int i = 0; i < m_ptsx.size(); i++) {
                        double shift_x = m_ptsx[i] - m_startX;
                        double shift_y = m_ptsy[i] - m_startY;

                        m_ptsx[i] = shift_x * cos(-m_startYaw) - shift_y * sin(-m_startYaw);
                        m_ptsy[i] = shift_x * sin(-m_startYaw) + shift_y * cos(-m_startYaw);
                    }
            }
}