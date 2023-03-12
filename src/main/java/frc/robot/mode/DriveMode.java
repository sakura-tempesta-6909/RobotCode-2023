package frc.robot.mode;

import frc.robot.State;
import frc.robot.State.DriveState;
import frc.robot.State.GrabHandState;
import frc.robot.State.MoveLeftAndRightArmState;
import frc.robot.State.RollerState;
import frc.robot.State.Hand.RotateState;
import frc.robot.subClass.Const;

public class DriveMode extends Mode {

    private static GrabGamePiecePhase phase = GrabGamePiecePhase.Phase1;
    private static int GrabCount = 0;

    @Override
    public void changeMode() {
        if (driveController.getStartButton()){
            State.mode = State.Modes.k_drive;
        }
        if (driveController.getBackButton()){
            State.mode = State.Modes.k_arm;

        }
    }

    @Override
    public void changeState() {
        State.driveXSpeed = -driveController.getLeftY();
        State.driveZRotation = -driveController.getRightX();
        State.driveState = DriveState.s_fastDrive;

        //RT: intake, LT: outtake
        if (driveController.getRightTriggerAxis() > 0.5) {
            State.intakeState = RollerState.s_intakeGamePiece;
        } else if (driveController.getLeftTriggerAxis() > 0.5) {
            State.intakeState = RollerState.s_outtakeGamePiece;
        } else {
            State.intakeState = RollerState.s_stopRoller;
        }

        //YボタンでBasicPositionに戻る, XボタンでゲームピースをつかんでbasicPositionに戻る

        if (driveController.getXButtonPressed()) {
            phase = GrabGamePiecePhase.Phase1;
        }

        if (driveController.getYButton()) {
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Arm.BasicPositionHeight;
            State.Arm.targetDepth = Const.Arm.BasicPositionDepth;
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
            State.rotateState = RotateState.s_turnHandBack;
        } else if (driveController.getXButton()) {
            switch (phase) {
                case Phase1:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.Arm.BasicPositionHeight;
                    State.Arm.targetDepth = Const.Arm.BasicPositionDepth;
                    State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
                    State.rotateState = RotateState.s_turnHandBack;
                    if (State.Arm.isArmAtTarget) {
                        phase = GrabGamePiecePhase.Phase2;
                    }
                case Phase2:
                    State.Hand.grabHandState = GrabHandState.s_releaseHand;
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armIntakeHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armIntakeDepth;
                    if (State.Arm.isArmAtTarget) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    State.Hand.grabHandState = GrabHandState.s_grabHand;
                    GrabCount++;
                    if (GrabCount >= 20) {
                        phase = GrabGamePiecePhase.Phase4;
                        GrabCount = 0;
                    }
                    break;
                case Phase4:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.Arm.BasicPositionHeight;
                    State.Arm.targetDepth = Const.Arm.BasicPositionDepth;
                    break;
            }
        }

        if (driveController.getAButton()) {
            State.driveState = DriveState.s_apriltagTracking;
        } else if (driveController.getBButton()) {
            State.driveState = DriveState.s_targetTracking;
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
    }

}
