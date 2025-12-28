#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

namespace argos {

   class ControllerBug1 : public CCI_Controller {

   public:

      ControllerBug1() {}

      virtual ~ControllerBug1() {}

      void Init(TConfigurationNode& t_tree) override;

      void ControlStep() override;

   private:
      enum Estate {
         STATE_FORWARD,
         STATE_OBSTABLE_FOLLOWING,
         STATE_OBSTACLE_RETURN,
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
      bool isObstacleAhead() const;
      double getRightReading() const;
      CVector2 m_cHitObstacle;
      CVector2 m_cClosestPointToTarget;
      Real m_cClosestDistance;
      bool m_obtacleCInvestagted;

      CVector2 getRobotPosition() const;
      CRadians getRobotHeading() const;
      void followObstacle() const;
      bool completedLoop() const;
      void updateClosestPoint();
      bool navigateToPoint(const CVector2& targetPoint);
   };
}