#include <stdexcept>
#include <math.h>
#include "Car.hpp"
#include <iostream>


const double INC_SPEED = 0.05 * 2.24; 


using namespace std;

namespace planning{ 

        Car::Car()
        :m_aheadSave(true),
        m_leftSave(true),
        m_rightSave(true),
        m_lane(1),
        m_desiredSpeed(0.0),
        m_trueSpeed(0.0)
        {
        }

        Car::~Car(){

        }

        void Car::setSD(double s){
            m_s = s;
            m_aheadSave = true;
            m_leftSave = (m_lane == 0)?false:true;
            m_rightSave = (m_lane == 2)?false:true;
            m_desiredSpeed = m_trueSpeed;

        }

        void Car::updateState(){
            if (!m_aheadSave && m_trueSpeed >= m_desiredSpeed) {
                m_trueSpeed -= INC_SPEED; 
                if ( m_leftSave ){
                     m_lane--;
                } else if (m_rightSave) {
                    m_lane++;
                }
            } 
            else if (m_trueSpeed < 49.5) {
                m_trueSpeed += INC_SPEED;
            }
        }


        void Car::updateSavety(vector<vector<double>> sensor_fusion, int latency){
            for(int i = 0; i < sensor_fusion.size(); i++) {
                double  d      = sensor_fusion[i][6];
                if(d<0 || d>12) continue;
                double  vx     = sensor_fusion[i][3];
                double  vy     = sensor_fusion[i][4];
                double  speed  = sqrt(vx * vx + vy * vy);
                double  s      = sensor_fusion[i][5];
                int     lane   = getLane(d);


                double current_s = s + ((double)latency * 0.02 * speed);

                if ( lane == m_lane) 
                {
                    if ( !checkSaveDistance(current_s, m_s) ) {
                        m_desiredSpeed = speed * 2.24;
                        m_aheadSave = false;
                    }
                }

                else if (lane == m_lane - 1) 
                {
                    if ( !checkSaveDistance(s , m_s - 5) 
                    || !checkSavetyBehind(s, m_s-5)) m_leftSave = false;
                }
                
                else if (lane == m_lane + 1) 
                {
                    if (!checkSaveDistance(s, m_s - 5 ) 
                    || !checkSavetyBehind(s, m_s-5)) m_rightSave = false;
                }

            }
        }

        bool Car::checkSaveDistance(double otherCarS, double ownS){
            bool save =  ((otherCarS > ownS) && (otherCarS - ownS) < 30)?false:true;
            return save;
        }

        bool Car::checkSavetyBehind(double otherCarS, double ownS){
            bool save =  ((otherCarS < ownS) && (ownS - otherCarS) < 15)?false:true;
            return save;
        }

        int Car::getLane(double d){
            if ( d >=0 && d < 4 ) return 0;
            if ( d >=4 && d < 8 ) return 1;
            if ( d >=8 && d < 12 ) return 2;
            else {
                throw "invalid d!"; 
            }
        }
}
