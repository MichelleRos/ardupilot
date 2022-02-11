#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */
#define HGT -1

 bool ModeAuto::init(bool ignore_checks){
    step = 1;
    target_pos = {0,0,1};
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Starting mission.");
    return true;
 }

//Runs the main loiter controller
void ModeAuto::run()
{
    //Step 0 is home, -1 means finished (or stop), 1 onwards is the waypoints
    float distsq = blimp.pos_ned.distance_squared(target_pos);
    if ((distsq < sq(g.wpnav_radius)) && step != -1 && step != 0){
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Finished waypoint %d, distance is %f, pos_ned=%f,%f,%f, target was=%f,%F,%f", step, sqrt(distsq), blimp.pos_ned.x, blimp.pos_ned.y, blimp.pos_ned.z, target_pos.x, target_pos.y, target_pos.z);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Finished waypoint %d", step);
        step++;
    }
    //else GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Doing step %d, distance sq is %f, pos_ned=%f,%f,%f, target=%f,%F,%f", step, sqrt(distsq), blimp.pos_ned.x, blimp.pos_ned.y, blimp.pos_ned.z, target_pos.x, target_pos.y, target_pos.z);

    //For square
    switch (step) {
        case 1:
            target_pos = {0,0,HGT};
            break;
        case 2:
            target_pos = {3,0,HGT};
            break;
        case 3:
            target_pos = {3,3,HGT};
            break;
        case 4:
            target_pos = {0,3,HGT};
            break;
        case 5:
            target_pos = {0,0,HGT};
            break;
        case 6:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Finished square mission. Starting circle.");
            break;
        default:
            break;
    }//*/

    //For circle
    float xes[] = {0.6, 0.1, 0.7, 2.3, 3.8, 4.4, 3.8, 2.3, 0.6, 0};
    float ys[] =  {0.7, 2.3, 3.7, 4.4, 3.6, 2.2, 0.8, 0.1, 0.7, 0};
    if (step>6 && step<17){
        target_pos = {xes[step-7],ys[step-7],HGT};
    }
    else if (step<=6){} //Doing square
    else {
        step = -1;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Finished circle mission.");
    }//*/

    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
}
