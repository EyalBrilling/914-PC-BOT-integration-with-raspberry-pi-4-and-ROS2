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
    for(int i=0;i<=1000;i++){
    wbr914.SetVelocityInTicks(30000,30000);
    wbr914.UpdateM3();
    }
    sleep(1);
    wbr914.SetVelocity(0,0);
    wbr914.MainQuit();
    //wbr914.SetVelocity(1,1);

}