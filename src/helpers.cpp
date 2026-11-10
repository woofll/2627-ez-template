#include "main.h"
//void functionName(){
// description of the function
//}
extern Drive chassis;

void toggle(){
chassis.pid_drive_set(-8, 120);
 chassis.pid_wait();

chassis.pid_drive_set(13, 127);
chassis.pid_wait();

chassis.pid_drive_set(-8, 120); 
chassis.pid_wait();
 
chassis.pid_drive_set(13, 127);
chassis.pid_wait();
}