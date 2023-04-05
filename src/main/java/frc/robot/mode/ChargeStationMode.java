package frc.robot.mode;

import java.util.Map;

import frc.robot.states.ArmState;
import frc.robot.states.DriveState;
import frc.robot.states.HandState;
import frc.robot.subClass.*;
import frc.robot.states.*;
import frc.robot.consts.ArmConst;
import frc.robot.consts.GrabGamePiecePhaseConst;
import frc.robot.subClass.Tools;;

public class ChargeStationMode extends Mode{
// 正式名称は「ワクワクドキドキ神様お願いブレイクモード」

    private static GrabGamePiecePhase phase = GrabGamePiecePhase.Phase1;
    private static int GrabCount = 0;
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
        DriveState.isMotorBrake = true;
        DriveState.xSpeed = -1 * driveController.getLeftY();
        DriveState.zRotation = -1 * driveController.getRightX();

        DriveState.driveState = DriveState.DriveStates.s_fastDrive;

        if (joystick.getRawButton(1)) {
            switch (phase) {
                case Phase1:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    Util.Calculate.setInitWithRelay();
                    ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                    if (ArmState.isAtTarget()) {
                        HandState.targetAngle = HandState.actualHandAngle + 90;
                        phase = GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = ( ArmConst.InitialHeight+GrabGamePiecePhaseConst.armCubeGrabHeight) / 2 +5;
                    ArmState.targetDepth = GrabGamePiecePhaseConst.armCubeGrabDepth;
                    HandState.rotateState = HandState.RotateStates.s_moveHandToSpecifiedAngle;
                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = GrabGamePiecePhaseConst.armCubeGrabHeight;
                    ArmState.targetDepth = GrabGamePiecePhaseConst.armCubeGrabDepth;
                    break;
            }


        } else if (joystick.getRawButton(2)) {
            // すべてBasicPositionに戻る
            ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
            HandState.rotateState = HandState.RotateStates.s_turnHandBack;
            Util.Calculate.setInitWithRelay();
        }

        if (joystick.getRawButtonPressed(1)) {
            ArmState.resetPidController = true;
            phase = GrabGamePiecePhase.Phase1;
        } else if (joystick.getRawButtonPressed(2)) {
            ArmState.resetPidController = true;
        }

        // ターゲット座標からターゲットの角度を計算する
        Map<String, Double> targetAngles = Tools.calculateAngles(ArmState.targetDepth, ArmState.targetHeight);
        Double target = targetAngles.get("RootAngle");
        if(target != null) {
            ArmState.targetRootAngle = target;
        } else {
            ArmState.targetRootAngle = ArmState.actualRootAngle;
        }
        target = targetAngles.get("JointAngle");
        if(target != null) {
            ArmState.targetJointAngle = target;
        } else {
            ArmState.targetJointAngle = ArmState.actualJointAngle;
        }
    }
    enum GrabGamePiecePhase {
        //basicPositionに移動する
        Phase1,
        //ハンドを開ける, アームを下げる
        Phase2,
        //ハンドを閉める
        Phase3,
        //アームを上げる
        Phase4,
        Phase5,
        Phase6,
        Phase7,
        Phase8,
    }
}