#include "wbr914_minimal.h"
int main(){
    wbr914_minimal wbr914;

    wbr914.MainSetup();
    bool test = wbr914.EnableMotors(true);  
    if (test){
        printf("good");
    }
    // sleep(5);
    wbr914.MainQuit();
    //wbr914.SetVelocity(1,1);

}