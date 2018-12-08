#include <stdexcept>
#include <math.h>
#include <vector>

using namespace std;

namespace planning{
class Car{
    public:
        bool m_aheadSave;
        bool m_leftSave;
        bool m_rightSave;

        int m_lane;

        double m_s;
        double m_desiredSpeed;
        double m_trueSpeed;
        
        Car();
        ~Car();
        
        void setSD(double s);
        void updateState();

        void updateSavety(vector<vector<double>> sensor_fusion, int latency);


    private:
        bool checkSaveDistance(double otherCarS, double ownS);
        bool checkSavetyBehind(double otherCarS, double ownS);

        int getLane(double d);

};
}