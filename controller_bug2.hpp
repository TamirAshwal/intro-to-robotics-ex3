// 209374867
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

namespace argos {

   class ControllerBug2 : public CCI_Controller {

   public:

      ControllerBug2() {}

      virtual ~ControllerBug2() {}

      void Init(TConfigurationNode& t_tree) override;

      void ControlStep() override;

   private:
   enum Estate {
         STATE_FORWARD,
         STATE_OBSTABLE_FOLLOWING,
         STATE_TARGET,
      };
      Estate m_eState;
      
      /* Sensors and Actuators */
      CCI_PiPuckDifferentialDriveActuator* m_pcWheels = nullptr;
      CCI_PiPuckColorLEDsActuator* m_pcColoredLEDs = nullptr;
      CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera = nullptr;
      CCI_PiPuckRangefindersSensor* m_pcRangefinders = nullptr;
      CCI_PiPuckSystemSensor* m_pcSystem = nullptr;
      CCI_PositioningSensor* m_pcPositioning = nullptr;
      CVector3 m_cTargetPosition;
      CVector2 m_startingPos;
      CVector2 m_cMLineDirection;     
      CRadians m_mLineHeading;   
      CVector2 m_hitPoint;   
      Real m_distanceAtHit; 
      // functions
      CVector2 getRobotPosition() const;
      CRadians getRobotHeading() const;
      bool reachedTarget() const;
      void targetState();
      void moveToTarget();
      bool isObstacleAhead() const;
      void toObstacleFollowState();
      double getRightReading() const;
      void followObstacle();
      void obstacleFollowState();
      bool isOnMLine() const;
   };
}
