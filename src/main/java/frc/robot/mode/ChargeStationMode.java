package frc.robot.mode;

import java.util.Map;

import frc.robot.States.State;
import frc.robot.subClass.*;

public class ChargeStationMode extends Mode{
// 正式名称は「ワクワクドキドキ神様お願いブレイクモード」
    @Override
    public void changeMode() {
        if (driveController.getStartButtonPressed()) {
            State.mode = State.Modes.k_drive;
        } else if (driveController.getBackButtonPressed()) {
            State.mode = State.Modes.k_arm;
        } else if (driveController.getLeftBumperPressed() && driveController.getPOV() == 225) {
            State.mode = State.Modes.k_config;
        }
    }

    @Override
    public void changeState() {
        State.Drive.isMotorBrake = true;
        State.Drive.xSpeed = -1 * driveController.getLeftY();
        State.Drive.zRotation = -1 * driveController.getRightX();

        State.Drive.state = State.Drive.States.s_fastDrive;

        if (joystick.getRawButton(1)) {
            if (State.Arm.targetHeight < -20) {
                State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                State.Arm.targetHeight = 0;
                State.Arm.targetDepth = 60;
            } else {
                State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                State.Arm.targetHeight = Const.GrabGamePiecePhase.armCubeIntakeHeight;
                State.Arm.targetDepth = Const.GrabGamePiecePhase.armCubeIntakeDepth;
            }
           
        } else if (joystick.getRawButton(2)) {
            // すべてBasicPositionに戻る
            if (State.Arm.targetHeight < -20) {
                State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                State.Arm.targetHeight = 0;
                State.Arm.targetDepth = 60;
            } else {
                State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                State.moveLeftAndRightArmState = State.MoveLeftAndRightArmState.s_movetomiddle;
                State.Hand.rotateState = State.Hand.RotateState.s_turnHandBack;
                State.Arm.targetHeight = Const.Arm.InitialHeight;
                State.Arm.targetDepth = Const.Arm.InitialDepth;
            }
          
        }

        if (joystick.getRawButtonPressed(1)) {
            State.Arm.resetPidController = true;
        } else if (joystick.getRawButtonPressed(2)) {
            State.Arm.resetPidController = true;
        }

          // ターゲット座標からターゲットの角度を計算する
          Map<String, Double> targetAngles = Tools.calculateAngles(State.Arm.targetDepth, State.Arm.targetHeight);
          Double target = targetAngles.get("RootAngle");
          if(target != null) {
              State.Arm.targetRootAngle = target;
          } else {
              State.Arm.targetRootAngle = State.Arm.actualRootAngle;
          }
          target = targetAngles.get("JointAngle");
          if(target != null) {
              State.Arm.targetJointAngle = target;
          } else {
              State.Arm.targetJointAngle = State.Arm.actualJointAngle;
          }
    }
}
