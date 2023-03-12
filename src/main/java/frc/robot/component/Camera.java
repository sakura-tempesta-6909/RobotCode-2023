package frc.robot.component;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.State;
import frc.robot.subClass.Const;

public class Camera implements Component {
    public Camera() {

    }

    public void calculation() {
        //角度を求める
        State.cameraCenterWidth = SmartDashboard.getNumber("CenterX", 0);
        State.cameraCenterHeight = SmartDashboard.getNumber("CenterY", 0);
        State.aprilTagAngleWidth = Math.toDegrees(Math.atan(State.cameraCenterWidth  / Const.Calculation.Camera.FocalLengthWeight));
        State.aprilTagAngleHeight = Math.toDegrees(Math.atan(State.cameraCenterHeight / Const.Calculation.Camera.FocalLengthHeight));
        SmartDashboard.putNumber("AngleX", State.aprilTagAngleWidth);
        SmartDashboard.putNumber("AngleY", State.aprilTagAngleHeight);

        //距離を求める
        double angleToGoalDegrees = Const.Calculation.Camera.CameraMountAngleDegrees + State.aprilTagAngleHeight;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
        State.cameraToTag = (Const.Calculation.Camera.GoalHeight - Const.Calculation.Camera.CameraLensHeight) / Math.tan(angleToGoalRadians);
        State.armToTag = State.cameraToTag - Const.Calculation.Camera.CameraToArm;
        SmartDashboard.putNumber("Distance", State.cameraToTag);


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
