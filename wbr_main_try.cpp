#include "wbr914_minimal.h"
int main(){
    wbr914_minimal wbr914;

    wbr914.MainSetup();
    wbr914.UpdateM3();
    bool test = wbr914.EnableMotors(true);  
    wbr914.UpdateM3();
    if (test){
        printf("good\n");
    }
    //wbr914.SetVelocity(10,10);
    wbr914.SetContourMode( VelocityContouringProfile );
    wbr914.SetVelocityInTicks(10,10);
    sleep(1);
    wbr914.SetVelocity(0,0);
    wbr914.MainQuit();
    //wbr914.SetVelocity(1,1);

}