package frc.robot.component;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.CameraState;
import frc.robot.states.State;
import frc.robot.consts.CameraConst;

public class Camera implements Component {
    public Camera() {

    }

    public void calculation() {
        //角度を求める
        CameraState.cameraCenterWidth = SmartDashboard.getNumber("CenterX", 0);
        CameraState.aprilTagAngleWidth = Math.toDegrees(Math.atan(CameraState.cameraCenterWidth  / CameraConst.FocalLengthWeight));
        CameraState.aprilTagAngleHeight = Math.toDegrees(Math.atan(CameraState.cameraCenterHeight / CameraConst.FocalLengthHeight));

        //距離を求める
        double angleToGoalDegrees = CameraConst.CameraMountAngleDegrees + CameraState.aprilTagAngleHeight;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
        CameraState.cameraToTag = (CameraConst.GoalHeight - CameraConst.CameraLensHeight) / Math.tan(angleToGoalRadians);
        CameraState.armToTag = CameraState.cameraToTag - CameraConst.CameraToArm;



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
