package frc.robot.component;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.CameraState;
import frc.robot.states.State;
import frc.robot.subClass.Const;

public class Camera implements Component {
    public Camera() {

    }

    public void calculation() {
        //角度を求める
        CameraState.cameraCenterWidth = SmartDashboard.getNumber("CenterX", 0);
        CameraState.aprilTagAngleWidth = Math.toDegrees(Math.atan(CameraState.cameraCenterWidth  / Const.Calculation.Camera.FocalLengthWeight));
        CameraState.aprilTagAngleHeight = Math.toDegrees(Math.atan(CameraState.cameraCenterHeight / Const.Calculation.Camera.FocalLengthHeight));
        SmartDashboard.putNumber("AngleX", CameraState.aprilTagAngleWidth);
        SmartDashboard.putNumber("AngleY", CameraState.aprilTagAngleHeight);

        //距離を求める
        double angleToGoalDegrees = Const.Calculation.Camera.CameraMountAngleDegrees + CameraState.aprilTagAngleHeight;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
        CameraState.cameraToTag = (Const.Calculation.Camera.GoalHeight - Const.Calculation.Camera.CameraLensHeight) / Math.tan(angleToGoalRadians);
        CameraState.armToTag = CameraState.cameraToTag - Const.Calculation.Camera.CameraToArm;
        SmartDashboard.putNumber("cameraToTag", CameraState.cameraToTag);
        SmartDashboard.putNumber("armToTag", CameraState.armToTag);


    }
    @Override
    public void autonomousInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void disabledInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void testInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void readSensors() {
        calculation();
    }


    @Override
    public void applyState() {
        // TODO Auto-generated method stub

    }

}
