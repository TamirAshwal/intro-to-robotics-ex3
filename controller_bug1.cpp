// 209374867
#include "controller_bug1.hpp"
// delete later
#include <iostream>
// end delete later

namespace argos {
   
   /****************************************/
   /****************************************/
   
   void ControllerBug1::Init(TConfigurationNode& t_tree) {
      /* Get the actuators and sensors */
      m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
      m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
      m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      m_pcCamera->Enable();
      m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
      m_pcPositioning->Enable();
      
      // read target position from .argos file into m_cTargetPosition
      TConfigurationNode& tTargetNode = GetNode(t_tree, "target_position");
      Real targetX, targetY;
      GetNodeAttribute(tTargetNode, "x", targetX);
      GetNodeAttribute(tTargetNode, "y", targetY);
      m_cTargetPosition.Set(targetX, targetY, 0.0);

      /* your Init code here */
      m_pcWheels->SetLinearVelocity(.154f, .154f);
      m_pcColoredLEDs->SetRingLEDs(CColor::BLUE);
      m_eState = STATE_FORWARD;
      m_cHitObstacle = CVector2(0, 0);
      m_cClosestPointToTarget = CVector2(0, 0);
      // big starting value
      m_cClosestDistance = 10000;
      m_obtacleCInvestagted = false;
      m_nStepsSinceHit = 0;
      }
   
   void ControllerBug1::ControlStep() {
      /* your ControlStep code here */

     
      switch(m_eState){
         case STATE_FORWARD:{
            CVector2 currentPos = getRobotPosition();
            CVector2 targetPos(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
            Real distToTarget = (currentPos - targetPos).Length();
   
   if (distToTarget < 0.10) { 
      m_eState = STATE_TARGET;
      m_pcColoredLEDs->SetRingLEDs(CColor::GREEN);
      m_pcWheels->SetLinearVelocity(0.0, 0.0);
      std::cout << "========================================" << std::endl;
      std::cout << "TARGET REACHED!" << std::endl;
      std::cout << "Final position: (" << currentPos.GetX() 
                << ", " << currentPos.GetY() << ")" << std::endl;
      std::cout << "========================================" << std::endl;
      return;
   }
            if(isObstacleAhead()){
               // obtacle detected go to obtacle follow and change light
               m_cHitObstacle = getRobotPosition();
               m_obtacleCInvestagted = false;
               m_nStepsSinceHit = 0;
               m_eState = STATE_OBSTABLE_FOLLOWING;
               m_pcColoredLEDs->SetRingLEDs(CColor:: YELLOW);
               std::cout << "Obstacle detected." << std::endl;
               return;
            }
            // get the current location and heading
            CVector3 cCurrentPos3D = m_pcPositioning->GetReading().Position;
            CRadians cZ, cY, cX;
            m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
            cZ.UnsignedNormalize();
            CVector2 cCurrentPosition(cCurrentPos3D.GetX(), cCurrentPos3D.GetY());
            CVector2 cTargetPosition(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
   
            // calcaulte the vector from the robot to the target
            CVector2 cToTarget = cTargetPosition - cCurrentPosition;
      
            // calculate the heading needed
            CRadians cRequiredHeading = cToTarget.Angle();
            cRequiredHeading.UnsignedNormalize();
      
            // calcualte the error
            CRadians cError = (cRequiredHeading - cZ).SignedNormalize();
      
            // check if the heading is correct
            const Real epsilon = 0.05;
            const bool bHeadingIsCorrect = Abs(cError.GetValue()) < epsilon;
            // if the heading is not correct turn until it's correct
            if (!bHeadingIsCorrect) {
               if (cError.GetValue() > 0) {
                  m_pcWheels->SetLinearVelocity(-.01f, .01f);  
               }
               else {
                  m_pcWheels->SetLinearVelocity(.01f, -.01f);  
               }
               }
            // heading is correct go to the target
            else {
               m_pcWheels->SetLinearVelocity(.154f, .154f);
            }   
               
            break;
      
         }
         case STATE_OBSTABLE_FOLLOWING:{
            // your obstacle following code here
            updateClosestPoint();
            m_nStepsSinceHit++; 
         
            if (m_nStepsSinceHit  > 50 && !m_obtacleCInvestagted && completedLoop()) {
               m_obtacleCInvestagted = true;
               m_eState = STATE_OBSTACLE_RETURN;
               m_pcColoredLEDs->SetRingLEDs(CColor::YELLOW);
               m_pcWheels->SetLinearVelocity(0.0, 0.0);
               std::cout << "Completed full loop! Stopping." << std::endl;
               std::cout << "Started at: (" << m_cHitObstacle.GetX() 
                        << ", " << m_cHitObstacle.GetY() << ")" << std::endl;
               std::cout << "Current position: (" << getRobotPosition().GetX() 
                        << ", " << getRobotPosition().GetY() << ")" << std::endl;
               return;
            }

            
            if(isObstacleAhead()){
               m_pcWheels->SetLinearVelocity(-0.1f, 0.1f);
            }
            else{
               // follow the obstacle
               followObstacle();
               }
         break;
         }
         case STATE_OBSTACLE_RETURN:{
            CVector2 currentPos = getRobotPosition();
            Real distToClosest = (currentPos - m_cClosestPointToTarget).Length();
            if (distToClosest < 0.12) {  // 12 ס"מ טולרנס
               m_eState = STATE_FORWARD;
               m_pcColoredLEDs->SetRingLEDs(CColor::BLUE);
               
               std::cout << "========================================" << std::endl;
               std::cout << "Reached closest point! Stopping." << std::endl;
               std::cout << "Position: (" << currentPos.GetX() 
                        << ", " << currentPos.GetY() << ")" << std::endl;
               std::cout << "Distance to target from here: " 
                        << m_cClosestDistance << " meters" << std::endl;
               std::cout << "========================================" << std::endl;
               }
            else{
               if(isObstacleAhead()){
                  m_pcWheels->SetLinearVelocity(-0.1f, 0.1f);
               }
               else{
                  followObstacle();
               }
            }
            break;

            
         }
        
         case STATE_TARGET:{
            // your target reached code here
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
            m_pcColoredLEDs->SetRingLEDs(CColor::GREEN);
         break;
         }
         // end of switch case
      }
      // end of controller
      
   }
   // function to detect obstacle ahead just like we saw in the recitation
   bool ControllerBug1::isObstacleAhead() const {
      bool bObstacle = false;
      std::function<void(const CCI_PiPuckRangefindersSensor::SInterface&)> fCheckSensor =
         [&bObstacle](const auto& sensor) {
            const auto& [id, pos, ori, range] = sensor.Configuration;
            if (sensor.Label != 0 && sensor.Label != 7) return; 
            if (sensor.Proximity < range) {
               bObstacle = true;
            }
         };
         m_pcRangefinders->Visit((fCheckSensor));
         return bObstacle;
   }
   /*
   fucntion to check distance of an obstacle to thr right simmilar to obstacle ahead but this one if for the right side of the robot
   */ 
   double ControllerBug1:: getRightReading() const{
      double minDistance = 100.0f;
      std::function<void(const CCI_PiPuckRangefindersSensor::SInterface&)> fCheckSensor =
         [&minDistance] (const auto& sensor){
         const auto&[id, pos, ori, range] = sensor.Configuration;
         if(sensor.Label !=5 && sensor.Label !=6) return;
         if(sensor.Proximity < range && sensor.Proximity < minDistance){
            minDistance = sensor.Proximity;
         }
      };
      m_pcRangefinders->Visit(fCheckSensor);
      return minDistance;
   }
   // function that returns the current position
   CVector2 ControllerBug1::getRobotPosition() const{
      const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
      return CVector2(reading.Position.GetX(), reading.Position.GetY());
   }
   CRadians ControllerBug1::getRobotHeading() const{
      CRadians cHeading, cPitch, cRoll;
      const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
      reading.Orientation.ToEulerAngles(cHeading, cPitch, cRoll);
      return cHeading;
   }
   void ControllerBug1:: followObstacle() const{
      // optimal distance from the wall
      const float TARGET_DIST = 0.10f; 
      const float K_P = 2.5f; 
      // speed for obstacle following       
      const float BASE_SPEED = 0.10f; 
      double currentDist = getRightReading();
      // חישוב השגיאה: המרחק שאנחנו רוצים פחות מה שיש בפועל
      double error = TARGET_DIST - currentDist; 
      float turnCorrection = K_P * error;
      m_pcWheels->SetLinearVelocity(BASE_SPEED + turnCorrection, BASE_SPEED - turnCorrection);
   }
   // fucntion that check if the current position if the poistion in which we hit the obstacle if so return true otherwise false
   bool ControllerBug1:: completedLoop() const{
      CVector2 currPosition = getRobotPosition();
      Real distanceToHitPoint = (currPosition - m_cHitObstacle).Length();
      const Real epsilon = 0.15;
      return distanceToHitPoint < epsilon;
   }
   // fucntion that updated the closest point and the distance to the target
   void ControllerBug1::updateClosestPoint(){
      CVector2 currentPos = getRobotPosition();
      CVector2 targetPos(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
      Real currentDistance = (currentPos - targetPos).Length();
      // if we found a closer point update the closest point and distance
      if (currentDistance < m_cClosestDistance) {
         m_cClosestDistance = currentDistance;
         m_cClosestPointToTarget = currentPos;
      }
   }





   /****************************************/
   /****************************************/
   
   REGISTER_CONTROLLER(ControllerBug1, "controller_bug1");
   
}