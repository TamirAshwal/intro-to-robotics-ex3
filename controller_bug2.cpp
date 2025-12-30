// 209374867
#include "controller_bug2.hpp"

namespace argos {
    
    /****************************************/
    /****************************************/
    
    void ControllerBug2::Init(TConfigurationNode& t_tree) {
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
        m_startingPos = getRobotPosition();
        CVector2 targetPos(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
         CVector2 toTarget = targetPos - m_startingPos;
        // calculate the m line 
        m_mLineHeading = toTarget.Angle();
        m_mLineHeading.UnsignedNormalize();
        m_eState = STATE_FORWARD;
        m_pcColoredLEDs->SetRingLEDs(CColor::BLUE);
        
    }
    void ControllerBug2::ControlStep() {
        /* your ControlStep code here */
        switch(m_eState){
            case STATE_FORWARD:{
                // check if we reached the target
                if(reachedTarget()){
                    targetState();
                    return;
                }
                //check if there is an obstacle ahead
                if(isObstacleAhead()){
                    // if so go to obstacle follow state
                    toObstacleFollowState();
                    return;
                }
                // if there is no obstacle move to target along the m line
                moveToTarget();
                break;
            }
            // obstacle following state
            case STATE_OBSTABLE_FOLLOWING:{
                obstacleFollowState();
                break;
            }
            // target reached state
            case STATE_TARGET:{
                targetState();
                break;
            }
        }
        // end of controll step
    }
    // just like in bug1 get the position from the positioning sensor like we saw in the recitation
    CVector2 ControllerBug2::getRobotPosition() const{
        const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
        return CVector2(reading.Position.GetX(), reading.Position.GetY());
    }
    // just like in bug1 get the heading from the positioning sensor like we saw in the recitation
    CRadians ControllerBug2::getRobotHeading() const{
        CRadians cHeading, cPitch, cRoll;
        const CCI_PositioningSensor::SReading& reading = m_pcPositioning->GetReading();
        reading.Orientation.ToEulerAngles(cHeading, cPitch, cRoll);
        return cHeading;
    }
    // check if we reached the target meaning if our distance from the target is smaller then some epsilon
    bool ControllerBug2::reachedTarget() const{
        // geth the robot position
        CVector2 currPos = getRobotPosition();
        // get the target position
        CVector2 targetPos (m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
        // calculate the distance to the target
        Real distanceToTarget = (currPos - targetPos).Length();
        // check if the distance is smaller then some epsilon
        return distanceToTarget < .10f;
    }
    // function that handles the state if we reached the target 
    void ControllerBug2::targetState(){
        // keep the target state
        m_eState = STATE_TARGET;
        // stop the robot
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        m_pcColoredLEDs->SetRingLEDs(CColor::GREEN);
    }
    // fucntion to move the robot towards the target along the m line
    void ControllerBug2::moveToTarget(){   
        // get the current heading and normalize it
        CRadians currHeading = getRobotHeading();
        currHeading.UnsignedNormalize();
        // calcaulte the error between the current heading and the m line
        CRadians error = (m_mLineHeading - currHeading).SignedNormalize();
        // Check if heading is correct meaning is smaller then some epsilon 
        const Real epsilon = 0.05;
        const bool bHeadingIsCorrect = Abs(error.GetValue()) < epsilon;
        
        // if the heading is not correct turn until you reach the right heading 
        if (!bHeadingIsCorrect) {
            if (error.GetValue() > 0) {
                m_pcWheels->SetLinearVelocity(-.01f, .01f);  
            }
            else {
                m_pcWheels->SetLinearVelocity(.01f, -.01f);  
            }
        }
        // heading is correct move forward
        else {
            m_pcWheels->SetLinearVelocity(.154f, .154f);
        }   
    }
    // fucntion that checks is there is an obstacle ahead using the front sensors
    bool ControllerBug2::isObstacleAhead() const {
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
   // this function reset the hit point and the distance from the target when we hit an obstacle 
   void ControllerBug2::toObstacleFollowState(){
        // update the latsest hit point
        m_hitPoint = getRobotPosition();
        CVector2 targetPos(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
        m_distanceAtHit = (m_hitPoint - targetPos).Length();
        // change the state to obstacle follow
        m_eState= STATE_OBSTABLE_FOLLOWING;
        m_pcColoredLEDs->SetRingLEDs(CColor:: YELLOW);

   }
   // functions that handles the obstacle follow state
   void ControllerBug2::obstacleFollowState(){
        // check if we are on the m line 
        if(isOnMLine()){
        // if so we can move forward
            m_eState = STATE_FORWARD;
            m_pcColoredLEDs->SetRingLEDs(CColor::BLUE);
            return;
        }
        // if not we need to continue to follow the obstacle
        followObstacle();
   }
   // function to check if we are on the m line 
   bool ControllerBug2::isOnMLine() const{
    // get the current position and the target position 
        CVector2 currPos = getRobotPosition();
        CVector2 targetPos(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
        // calculate the distance from the robot to the m line
        CVector2 mLine = targetPos - m_startingPos;  
        CVector2 distanceLine = currPos - m_startingPos;   
        // calcaulte the cross product
        Real crossProduct = distanceLine.GetX() * mLine.GetY() - distanceLine.GetY() * mLine.GetX();
        // calculate m line length
        Real lineLength = mLine.Length();
        // calcualte the perpendicular distance from the robot to the m line
        Real distance = Abs(crossProduct) / lineLength;
        const Real epsilon = 0.10;  
        // if it is closer then some epsilon we are on the m line
        if (distance > epsilon) {
            return false;
        }
        /*
        important check if we made a progress meaning we got closer to the target
        to prevent infinite loop
        */ 
        Real currentDistToTarget = (currPos - targetPos).Length();
        return currentDistToTarget < m_distanceAtHit;
   }
   // just like bug1 check the sensors on the right liek we saw in the recitation
   double ControllerBug2::getRightReading() const{
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
   // function to follow the obstacle
   void ControllerBug2::followObstacle(){
    // if the obstacle is in front of the robot rotate in place
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
    
    /****************************************/
    /****************************************/
    
    REGISTER_CONTROLLER(ControllerBug2, "controller_bug2");
    
}
