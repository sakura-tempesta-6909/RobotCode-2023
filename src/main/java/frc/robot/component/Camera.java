package frc.robot.component;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.States.State;
import frc.robot.consts.CameraConst;

public class Camera implements Component {
    public Camera() {

    }

    public void calculation() {
        //角度を求める
        State.cameraCenterWidth = SmartDashboard.getNumber("CenterX", 0);
        State.cameraCenterHeight = SmartDashboard.getNumber("CenterY", 0);
        State.aprilTagAngleWidth = Math.toDegrees(Math.atan(State.cameraCenterWidth  / CameraConst.FocalLengthWeight));
        State.aprilTagAngleHeight = Math.toDegrees(Math.atan(State.cameraCenterHeight / CameraConst.FocalLengthHeight));
        SmartDashboard.putNumber("AngleX", State.aprilTagAngleWidth);
        SmartDashboard.putNumber("AngleY", State.aprilTagAngleHeight);

        //距離を求める
        double angleToGoalDegrees = CameraConst.CameraMountAngleDegrees + State.aprilTagAngleHeight;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
        State.cameraToTag = (CameraConst.GoalHeight - CameraConst.CameraLensHeight) / Math.tan(angleToGoalRadians);
        State.armToTag = State.cameraToTag - CameraConst.CameraToArm;
        SmartDashboard.putNumber("cameraToTag", State.cameraToTag);
        SmartDashboard.putNumber("armToTag", State.armToTag);


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
