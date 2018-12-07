#include "spline.h"
#include <vector>
#include <math.h>

using namespace std;
namespace planning{
class Planner{
    public:
        tk::spline m_spline;
        vector<double> m_ptsx;
        vector<double> m_ptsy;
        double m_startX;
        double m_startY;
        double m_startYaw;
        int m_planSize;

        Planner(double car_x, double car_y, double car_yaw, vector<double> previous_path_x, vector<double> previous_path_y);

        ~Planner();

        void calculateSplineWithWayPoints(vector<double> waypoint1, vector<double> waypoint2, vector<double> waypoint3);
        

        void plan(vector<double> &next_x_vals, vector<double> &next_y_vals, double speed);

        private:
            double deg2rad(double x);
            double rad2deg(double x);
            void shiftToStartPoint();
};

}