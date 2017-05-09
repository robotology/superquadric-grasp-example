# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Vezzani
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# idl.thrift
/**
* Property
*
* IDL structure to set/show advanced parameters.
*/
struct Bottle
{
} (
   yarp.name = "yarp::os::Bottle"
   yarp.includefile="yarp/os/Bottle.h"
  )

/**
* testingModule_IDL
*
* IDL Interface to \ref testing-module services.
*/

service experimentOne_IDL
{

    /* Send the current 2D blob
    *@return a bottle containing all the 2D points of the blob
    */
    Bottle  get_blob();

    /*
    * Set the streaming mode on/off 
    * (useful for visualization and tracking).
    * @param entry can be "on" or "off".
    * @return true/false on success/failure.
    */
    bool set_streaming_mode(1: string entry);
    
    /*
    * Set the name of the object
    * (stored by the object property collector).
    * @param entry is the name of the object
    * @return true/false on success/failure.
    */
    bool  set_object_name(1: string entry);

    /*
    * Ask to go to the next step, following the pipeline:
    * 1- compute superquadric
    * 2- compute pose
    * 3- ask the robot to move
    * @return true.
    */
    bool go_next();

    /*
    * The pipeline is restarted and is 
    * waiting the command "go_next" to start
    * from step 1.
    * @return true.
    */
    bool start_from_scratch();

    /*
    *If you want just to perform step 1.
    * @return true.
    */
    bool acquire_superq();
    
    /*
    *If you want just to perform step 2
    * (if step 1 has been performed).
    * @return true/false on success/failure.
    */
    bool compute_pose();

    /*
    *If you want just to perform step 3
    * (if step 2 has been performed).
    * @return true/false on success/failure.
    * @return true.
    */
    bool grasp_object();

    /*
    * Set the hand for pose computation.
    * @param entry can be "right", "left" or "both".
    * @return true/false on success/failure.
    */
    bool set_hand_for_computation(1: string h);
    
    /*
    * Get the hand for pose computation.
    * @return "right", "left" or "both".
    */
    string get_hand_for_computation();
    
    /*
    * Set the hand for moving.
    * @param entry can be "right" or "left".
    * @return true/false on success/failure.
    */
    bool set_hand_for_moving(1: string h);

    /*
    * Get the hand for pose computation.
    * @return "right", "left" or "both".
    */
    string get_hand_for_moving();

}


