#include "wbr914_minimal.h"
int main(){
    wbr914_minimal wbr914;

    wbr914.init_robot();
    bool test = wbr914.EnableMotors(true);  
    if (test){
        printf("good");
    }
    // sleep(5);
     wbr914.EnableMotors(false);
    //wbr914.SetVelocity(1,1);
}