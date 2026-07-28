/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */
#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H
#include <argos3/core/utility/math/vector2.h>
#include<numeric>
#include <algorithm>
/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotDiffusion : public CCI_Controller {

public:

   /* Class constructor. */
   CFootBotDiffusion();

   /* Class destructor. */
   virtual ~CFootBotDiffusion() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function calculates the angle of each section
    * the 'section' variable points to one of the 4 sections : 
    * frontLeft, backLeft, backRight, frontRight
    * The reading (vector length) of each sensor is set to '1' since if there are no obstacles, 
    * the reading is 0 which causes the vector to be zero and thus for angle to be undefined (defaults to 0)
    */
   CRadians SectionAngle(const std::vector<int>& section);

   /*
    * This function return the average of the readings of the six sensors of a section.
    * the 'section' variable points to one of the 4 sections : 
    * frontLeft, backLeft, backRight, frontRight
    */
   Real AverageReading(const std::vector<int>& section);

   /*
    * This function returns the angle of the sections with the lowest
    * density of obstacles, which means the section with the lowest reading.
    */
   CRadians LowestDensitySection();

   /*
    * This function returns true if the front of the robot is the current least
    * obstructed section, i.e has the lowest average reading.
    */
   bool IsFrontEmpty();

   /*
    * This function checks if an obstacle has been detected.
    * It finds the closest obstacle: the maximum reading across all sensors
    * and returns true if that reading reaches the maximum proximity tolerance
    * m_fDelta (obstacle detected) and false otherwise (path clear).
    */
   bool IsObstacleDetected(); 

   /*
    * This function moves the robot forward in a straight line.
    */
   void GoStraight();

   /*
    * This function turns the robot to the right in place without moving.
    */
   void GoRight();

   /*
    * This function turns the robot to the left in place without moving.
    */
   void GoLeft();

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}
   
   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}
private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real m_fDelta;
   /* Wheel speed. */
   Real m_fWheelVelocity;
   /* See the sensors positions here: https://github.com/ilpincy/argos3/blob/master/src/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h
   * frontLeft sensors: 0,1,2,3,4,5
   * backLeft sensors 6,7,8,9,10,1
   * backRight sensors 12,13,14,15,16,17
   * frontRight sensors 18,19,20,21,22,23
   */
   struct section{
      std::string name;
      CRadians angle;
      Real reading;
      std::vector<int> sensors;
   };
   /* [frontLeft, backLeft, backRight, frontRight]*/
   std::vector<section> m_sections;
   /* Check if we are currently executing a turn in the direction least dense section */
   bool m_bTurning;   
   /* The direction of the least dense section we are turning to */
   CRadians m_newDirection; 
};

#endif
