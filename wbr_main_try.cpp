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
    int32_t left;
    int32_t right;
    //wbr914.SetVelocity(10,10);
    wbr914.SetContourMode( VelocityContouringProfile );
    for(int i=0;i<=1000;i++){
    wbr914.SetVelocityInTicks(10000,10000);
    wbr914.UpdateM3();
    wbr914.GetVelocityInTicks(&left,&right);
    printf("left:%i right:%i Velocities ticks\n",left,right);
    }
    sleep(1);
    wbr914.SetVelocity(0,0);
    wbr914.MainQuit();
    //wbr914.SetVelocity(1,1);

}