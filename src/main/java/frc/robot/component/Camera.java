package frc.robot.component;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.Util;

public class Camera implements Component {
    public Camera() {

    }

    public void calculation() {
        //角度を求める
        State.cameraCenterWidth = Util.getConsole("CenterX", 0.0);
        State.cameraCenterHeight = Util.getConsole("CenterY", 0.0);
        State.aprilTagAngleWidth = Math.toDegrees(Math.atan(State.cameraCenterWidth  / Const.Calculation.Camera.FocalLengthWeight));
        State.aprilTagAngleHeight = Math.toDegrees(Math.atan(State.cameraCenterHeight / Const.Calculation.Camera.FocalLengthHeight));

        //距離を求める
        double angleToGoalDegrees = Const.Calculation.Camera.CameraMountAngleDegrees + State.aprilTagAngleHeight;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        State.cameraToTagCenterDepth = (Const.Calculation.Camera.GoalHeight - Const.Calculation.Camera.CameraLensHeight) / Math.tan(angleToGoalRadians);
        State.armRootToTagCenterDepth = State.cameraToTagCenterDepth - Const.Calculation.Camera.CameraToArmRootDepth;
//        SmartDashboard.putNumber("Distance", State.cameraToTagCenterDepth);

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
        State.Arm.SpecificTargetDepths.TopCube = State.armRootToTagCenterDepth + (58 - 41) + 43;
        State.Arm.SpecificTargetDepths.MiddleCube = State.armRootToTagCenterDepth + (58 - 41);
        State.Arm.SpecificTargetDepths.BottomCube = State.armRootToTagCenterDepth + (58 - 41) - 10;
    }


    @Override
    public void applyState() {
        // TODO Auto-generated method stub

    }

}
