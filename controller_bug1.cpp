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
      m_closestPointToTarget = CVector2(0, 0);
      // big starting value
      m_cClosestDistance = 10000;
      m_obstacleInvestagted = false;
      m_stepsSinceHit = 0;
      }
   
   void ControllerBug1::ControlStep() {
      /* your ControlStep code here */
      switch(m_eState){
         case STATE_FORWARD:{
            forwardState();            
            break;
      
         }
         case STATE_OBSTABLE_FOLLOWING:{
           
            obstacleFollowState();
         break;
         }
         case STATE_OBSTACLE_RETURN:{
            obstacleReturnState();
            break;
         }
         case STATE_TARGET:{
            toTargetState();
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
         m_closestPointToTarget = currentPos;
      }
   }
   void ControllerBug1:: forwardState(){
      // check if the robot reached the target is so change to target state
      if(reachedTarget()){
         toTargetState();
         return;
      }
      if(isObstacleAhead()){
         toObstacleFollowState();
         return;
      }
      moveToTarget();
   }
   bool ControllerBug1:: reachedTarget() const{
      CVector2 currPos = getRobotPosition();
      CVector2 targetPos (m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
      Real distanceToTarget = (currPos - targetPos).Length();
      return distanceToTarget < .10f;
   }
   void ControllerBug1:: toTargetState(){
      CVector2 currPos = getRobotPosition();
      m_eState = STATE_TARGET;
      m_pcWheels->SetLinearVelocity(0.0, 0.0);
      m_pcColoredLEDs->SetRingLEDs(CColor::GREEN);
   }
   void ControllerBug1:: toObstacleFollowState(){
      // update the latsest hit point
      m_cHitObstacle = getRobotPosition();
      // reset the the variables for the new obstacle
      m_obstacleInvestagted = false;
      m_stepsSinceHit = 0;
      m_eState= STATE_OBSTABLE_FOLLOWING;
      m_pcColoredLEDs->SetRingLEDs(CColor:: YELLOW);
   }
   void ControllerBug1::moveToTarget(){
      CRadians currHeading = getRobotHeading();
      currHeading.UnsignedNormalize();
      CVector2 currPos  = getRobotPosition();
      CVector2 targetPos(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());

      // calcaulte the vector from the robot to the target
      CVector2 toTarget = targetPos - currPos;
   
      // calculate the heading needed
      CRadians requiredHeading = toTarget.Angle();
      requiredHeading.UnsignedNormalize();
      // calcualte the error
      CRadians error = (requiredHeading - currHeading).SignedNormalize();
      // check if the heading is correct
      const Real epsilon = 0.05;
      const bool bHeadingIsCorrect = Abs(error.GetValue()) < epsilon;
      // if the heading is not correct turn until it's correct
      if (!bHeadingIsCorrect) {
         if (error.GetValue() > 0) {
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

   }
   void ControllerBug1::obstacleFollowState(){
      updateClosestPoint();
      m_stepsSinceHit++;
      // this checks if we moved aways a little bit from the hit point and completed the loop
      if(m_stepsSinceHit > 50 && !m_obstacleInvestagted && completedLoop()){
         toObstacleReturnState();
         return;
      }
      followObstacle();
   }
   void ControllerBug1:: followObstacle(){
      if(isObstacleAhead()){
          m_pcWheels->SetLinearVelocity(-0.1f, 0.1f);
      }
      else{
         // distance from the wall
         const float obstacleDistance = .10f; 
         const float K_P = 2.5f; 
         double currDistance = getRightReading();
         // calculate the error
         double error = obstacleDistance - currDistance; 
         float turnCorrection = K_P * error;
         m_pcWheels->SetLinearVelocity(.10f + turnCorrection, .10f - turnCorrection);
      }
   }
   void ControllerBug1:: toObstacleReturnState(){
      m_obstacleInvestagted = true;
      m_eState = STATE_OBSTACLE_RETURN;
      m_pcColoredLEDs->SetRingLEDs(CColor::YELLOW);
      
   }
   // this function checks if we reached to the closest point if so change to forward state else continue to follow the obstacle
   void ControllerBug1::obstacleReturnState(){
      Real epsilon = .12f;
      CVector2 currPos = getRobotPosition();
      Real distanceToClosest = (currPos - m_closestPointToTarget).Length();
      // if we are really close to the closeset point change to forward state
      if(distanceToClosest < epsilon)
          {
            m_eState = STATE_FORWARD;
            m_pcColoredLEDs->SetRingLEDs(CColor::BLUE);
            return;
          }
          // continue to follow the obstacle
          followObstacle();
   }
   /****************************************/
   /****************************************/
   
   REGISTER_CONTROLLER(ControllerBug1, "controller_bug1");
   
}