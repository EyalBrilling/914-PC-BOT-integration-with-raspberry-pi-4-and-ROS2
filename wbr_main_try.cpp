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
    int32_t leftVel;
    int32_t rightVel;
    int32_t leftPos;
    int32_t rightPos;
    //wbr914.SetVelocity(10,10);
    wbr914.SetContourMode( VelocityContouringProfile );
    for(int i=0;i<=1000;i++){
    wbr914.SetVelocityInTicks(10000,5000);
    wbr914.UpdateM3();
    wbr914.GetVelocityInTicks(&leftVel,&rightVel);
    printf("left:%i right:%i Velocities ticks\n",leftVel,rightVel);
    wbr914.GetPositionInTicks(&leftPos,&rightPos);
    printf("left:%i right:%i Positions in ticks\n",leftPos,rightPos);
    }
    sleep(1);
    wbr914.SetVelocity(0,0);
    wbr914.MainQuit();
    //wbr914.SetVelocity(1,1);

}