// #include "main.h"


// const int CLAWnumStates = 3; 
// int CLAWstates[CLAWnumStates] = {0, 90, 130};
// int CLAWcurrState = 0;
// int CLAWtarget = 0;

// void CLAWnextState() {
//   CLAWcurrState += 1;
//   if (CLAWcurrState = 3) {
//     CLAWcurrState = 0;
//   }

//   CLAWtarget = CLAWstates[CLAWcurrState];
// }

// // void CLAWliftControl() {
// //   double kp = 0.5;
// //   double error = CLAWtarget - (global::rotCascade.get_position()/100.0);
// //   double velocity = kp * error;
// //   global::cascadeLeft.move(velocity); 

// // }