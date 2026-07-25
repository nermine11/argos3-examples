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
                           ToRadians(m_cAlpha)){
   std::vector<int> sensors(24); // Original 24 elements 
   std::iota(sensors.begin(), sensors.end(), 0);

   frontLeft.assign(sensors.begin(),      sensors.begin() + 6);
   backLeft.assign(sensors.begin() + 6,  sensors.begin() + 12);
   backRight.assign(sensors.begin() + 12, sensors.begin() + 18);
   frontRight.assign(sensors.begin() + 18, sensors.end());
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
}

CVector2 CFootBotDiffusion::SumReadings(const std::vector<int>& section){
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < section.size(); ++i) {
      int index = section[i];
      cAccumulator += CVector2(tProxReads[index].Value, tProxReads[index].Angle);
   }
   cAccumulator /= section.size();
   return cAccumulator;

}

CRadians CFootBotDiffusion::LowDensitySection() {

   /* Get readings from each section */
   CVector2 cAccumulatorFrontRight = SumReadings(frontRight);
         LOG << "frontRight = " << cAccumulatorFrontRight << '\n';

   CVector2 cAccumulatorFrontLeft = SumReadings(frontLeft);
         LOG << "frontLeft = " << cAccumulatorFrontLeft << '\n';

   CVector2 cAccumulatorBackRight = SumReadings(backRight);
         LOG << "cAccumulatorBackRight = " << cAccumulatorBackRight << '\n';

   CVector2 cAccumulatorBackLeft = SumReadings(backLeft);
         LOG << "cAccumulatorBackLeft = " << cAccumulatorBackLeft << '\n';

   /* Find the section with the most space (less dense with obstacles)*/
   CVector2 cMinSection = std::min({cAccumulatorFrontRight, cAccumulatorFrontLeft, cAccumulatorBackRight, cAccumulatorBackLeft}, [](const CVector2& a, const CVector2& b) {
      return a.Length() < b.Length();     
   });
            LOG << "cMinSection = " << cMinSection << '\n';
            LOG << "cMinSection angle = " << cMinSection.Angle()<< '\n';

   return cMinSection.Angle();
}


/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight
    */
   // Direction of the obstacle field
   CRadians cAngle = cAccumulator.Angle(); 
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta ) {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      CRadians newDirection = LowDensitySection();
      LOG << "newDirection = " << newDirection << '\n';
      CRadians tolerance = ToRadians(CDegrees(5.0f));
      if(std::abs(newDirection.GetValue()) > tolerance.GetValue()){
         /* Turn, depending on the sign of the angle */
         // Section on the right -> Turn to the right
         if(newDirection.GetValue() > 0.0f) { 
            LOG << " move right \n ";
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
         }
         else {
            // Section on the left -> Turn to the left
            LOG << " move left \n ";
            m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
         }
      }else{
         /* Go straight */
         LOG << " go straight \n ";
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      }
      
   

   }
}
