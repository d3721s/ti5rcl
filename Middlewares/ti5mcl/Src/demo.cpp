#include "ti5mcl.hpp"
using namespace ti5mcl;
using namespace std;
int main()
{
    uint32_t c;
    float v;
    float p;
    ti5Motor M1(1, ti5Motor::reductionRatio101);
    ti5Motor M2(2, ti5Motor::reductionRatio101);
    ti5Motor M3(3, ti5Motor::reductionRatio81);
    ti5Motor M4(4, ti5Motor::reductionRatio81);
    ti5Motor M5(5, ti5Motor::reductionRatio51);
    while (1)
    {

        M1.home();
        M2.home();
        M3.home();
        M4.home();
        M5.home();
        this_thread::sleep_for(std::chrono::seconds(5));
        M1.quickGetCSP(&c,&v,&p);
        M1.moveAbsolute(M_PI/4);
        this_thread::sleep_for(std::chrono::seconds(5));
        M1.quickGetCSP(&c,&v,&p);
        M1.moveAbsolute(-M_PI/2);
        this_thread::sleep_for(std::chrono::seconds(5));
        M1.quickGetCSP(&c,&v,&p);

        M2.moveAbsolute(1.56192);
        M3.moveAbsolute(0.0647521);
        M4.moveAbsolute(0);
        M5.moveAbsolute(0.000899758);
        this_thread::sleep_for(std::chrono::seconds(5));

    }
    return 0;
}


