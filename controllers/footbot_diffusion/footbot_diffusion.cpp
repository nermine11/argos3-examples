/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* logging for debugging*/
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)),
   m_bTurning(false){
   /* Original 24 sensors */
   std::vector<int> sensors(24); 
   std::iota(sensors.begin(), sensors.end(), 0);
   /* Assign the sensors for each section, for now the angle and readings are 0 */
   m_sections.push_back({"frontLeft", CRadians::ZERO,
                       0.0, std::vector<int>(sensors.begin(), sensors.begin() + 6)});
   m_sections.push_back({"backLeft", CRadians::ZERO,
                       0.0, std::vector<int>(sensors.begin() + 6, sensors.begin() + 12)});
   m_sections.push_back({"backRight", CRadians::ZERO,
                       0.0, std::vector<int>(sensors.begin() + 12, sensors.begin() + 18)});
   m_sections.push_back({"frontRight", CRadians::ZERO,
                       0.0, std::vector<int>(sensors.begin() + 18, sensors.end())});
}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
   /* Add the fixed angle of each section */
   for(section&s : m_sections){
      s.angle = SectionAngle(s.sensors);
   }
}

/****************************************/
/****************************************/

CRadians CFootBotDiffusion::SectionAngle(const std::vector<int>& section) {
   /* Get readings from proximity sensors of this section */
   const CCI_FootBotProximitySensor::TReadings& tReads = m_pcProximity->GetReadings();
   CVector2 cDir;
   for(size_t i = 0; i < section.size(); ++i)  {
      int index = section[i];
     /* 
      * Use fixed length 1 because a section without obstacles has length 0 so 
      * the vector would sum to (0,0), whose angle is undefined. 
      * Length 1 ensures the vector is not null, so we can get a valid angle.
      */
      cDir += CVector2(1.0, tReads[index].Angle);   
   }
   return cDir.Angle();
}

/****************************************/
/****************************************/

Real CFootBotDiffusion::AverageReading(const std::vector<int>& section){
   /* Get readings from proximity sensors of this section */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < section.size(); ++i) {
      int index = section[i];
      cAccumulator += CVector2(tProxReads[index].Value, tProxReads[index].Angle);
   }
   /* Average them */
   cAccumulator /= section.size();
   return cAccumulator.Length();
}

/****************************************/
/****************************************/

CRadians CFootBotDiffusion::LowestDensitySection() {
   /* Get the average reading from each section */
   for(section& s : m_sections){
      s.reading = AverageReading(s.sensors);
   }
   /* Find the section with the most space (least dense with obstacles)*/
   const section* best = &m_sections[0];
   for(section& s : m_sections){
      if(s.reading < best->reading){
         best = &s;
      }
   }
   return best->angle;
}

/****************************************/
/****************************************/

bool CFootBotDiffusion::IsObstacleDetected(){
   /* Get readings from proximity sensors */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Find the closest obstacle (max reading) */
   Real fMaxProxRead = 0.0f;
   for(size_t i = 0; i < tProxReads.size(); ++i){
      if(tProxReads[i].Value > fMaxProxRead){
         fMaxProxRead = tProxReads[i].Value;
      }
   }
   return fMaxProxRead > m_fDelta;
}

/****************************************/
/****************************************/

bool CFootBotDiffusion::IsFrontEmpty(){
   /* Get the average reading of each section */
   /* It would be better to not use hardcoded indexes and use a map or a name lookup, but to keep it simple we used indexing */
   Real fFrontLeft  = AverageReading(m_sections[0].sensors); 
   Real fBackLeft   = AverageReading(m_sections[1].sensors);
   Real fBackRight  = AverageReading(m_sections[2].sensors);
   Real fFrontRight = AverageReading(m_sections[3].sensors);
   /* Front is composed of our two sections: front-right and front-left */
   Real front =(fFrontLeft + fFrontRight) / 2.0; 
   /* front is empty if it's the least obstructed direction */
   return (front <= fBackRight && front <= fBackLeft);
}

/****************************************/
/****************************************/

void CFootBotDiffusion::GoStraight(){
   m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CFootBotDiffusion::GoRight(){
   m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CFootBotDiffusion::GoLeft(){
   m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   /* If the closest obstacle is far enough and we are not currently turning towards 
    * the emptiest section after sensing an obstacle, continue going straight forward */
   if(!IsObstacleDetected() && !m_bTurning) {
      GoStraight();
      return;
   }
   /* Else, go to the emptiest section with the least obstacles */
   else {
      /* If I am not on the way to the section, find the section */
      if(!m_bTurning) {
         m_newDirection = LowestDensitySection();   
         /* Set to true so we keep turning toward this direction and 
         do not reclculate m_newDirection until we have fully reached it */
         m_bTurning = true;                        
      }
      /* If the front is empty, we conclude that we reached the emptiest
       section and stop turning */
      bool b_frontEmpty = IsFrontEmpty();
      /* Turn, depending on the sign of the angle, else go straight */
      if(!b_frontEmpty) {
         /* Section on the right -> Turn in place to the right */
         if(m_newDirection.GetValue() > 0.0f) { 
            GoRight();
         }
         /* Section on the left -> Turn in place to the left */
         else {
            GoLeft();   
         }
      }
      else{
         GoStraight();
         /* We reached the emptiest section, so we are not turning to it anymore */
         m_bTurning = false;
      }
   }
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")