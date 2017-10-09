// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_experimentOne_IDL
#define YARP_THRIFT_GENERATOR_experimentOne_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/os/Bottle.h>

class experimentOne_IDL;


/**
 * experimentOne_IDL
 * IDL Interface to \ref experiment-1 services.
 */
class experimentOne_IDL : public yarp::os::Wire {
public:
  experimentOne_IDL();
  /**
   *  Send the current 2D blob
   * @return a bottle containing all the 2D points of the blob
   */
  virtual yarp::os::Bottle get_blob();
  /**
   * Set the name of the object
   * (stored by the object property collector).
   * @param entry is the name of the object
   * @return true/false on success/failure.
   */
  virtual bool set_object_name(const std::string& entry);
  /**
   * Get the name of the object
   * @return a string with the name of the object.
   */
  virtual std::string get_object_name();
  /**
   * Ask to go to the next step, following the pipeline:
   * 1- compute superquadric
   * 2- compute pose
   * 3- ask the robot to move
   * @return true.
   */
  virtual bool go_next();
  /**
   * The pipeline is restarted and is
   * waiting the command "go_next" to start
   * from step 1.
   * @return true.
   */
  virtual bool start_from_scratch();
  /**
   * If you want just to perform step 1.
   *  @return true.
   */
  virtual bool acquire_superq();
  /**
   * If you want just to perform step 2
   *  (if step 1 has been performed).
   *  @return true/false on success/failure.
   */
  virtual bool compute_pose();
  /**
   * If you want just to perform step 3
   *  (if step 2 has been performed).
   *  @return true/false on success/failure.
   */
  virtual bool grasp_object();
  /**
   * Ask the robot to stop and go back
   *  to home position with the arm
   *  that is moving.
   *  @return true/false on success/failure.
   */
  virtual bool go_back_home();
  /**
   * Clear all the computed poses
   *  @return true.
   */
  virtual bool clear_poses();
  /**
   * Set the hand for pose computation.
   * @param entry can be "right", "left" or "both".
   * @return true/false on success/failure.
   */
  virtual bool set_hand_for_computation(const std::string& entry);
  /**
   * Get the hand for pose computation.
   * @return "right", "left" or "both".
   */
  virtual std::string get_hand_for_computation();
  /**
   * Set the hand for moving.
   * @param entry can be "right" or "left".
   * @return true/false on success/failure.
   */
  virtual bool set_hand_for_moving(const std::string& entry);
  /**
   * Get the hand for pose computation.
   * @return "right", "left" or "both".
   */
  virtual std::string get_hand_for_moving();
  /**
   * Get if automatic selection of the hand is on or off
   * @return "on" or "off".
   */
  virtual std::string get_automatic_hand();
  /**
   *  Set if automatic selection of the hand is on or off
   * @param entry can be "on" or "off"
   * @return true/false on success/failure.
   */
  virtual bool set_automatic_hand(const std::string& entry);
  /**
   * Set if to ask the filtered superquadric or not.
   * @param entry can be "on" or "off".
   * @return "on" or "off".
   */
  virtual bool set_filtered_superq(const std::string& entry);
  /**
   * Get if to ask the filtered superquadric or not.
   * @return true/false on success/failure.
   */
  virtual std::string get_filtered_superq();
  /**
   * Set if to reset the filtered superquadric or not.
   * @param entry can be "on" or "off".
   * @return "on" or "off".
   */
  virtual bool set_reset_filter(const std::string& entry);
  /**
   * Get if to reset the filtered superquadric or not.
   * @return true/false on success/failure.
   */
  virtual std::string get_reset_filter();
  /**
   * Set the current object class
   * @param entry the name of the object class
   * @return true/false on success/failure.
   */
  virtual bool set_object_class(const std::string& entry);
  /**
   * Get the current object class
   * @return the current object class.
   */
  virtual std::string get_object_class();
  virtual bool read(yarp::os::ConnectionReader& connection) YARP_OVERRIDE;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
