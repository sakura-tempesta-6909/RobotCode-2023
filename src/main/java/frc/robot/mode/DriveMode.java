package frc.robot.mode;

import frc.robot.State;
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
        if (driveController.getBackButtonPressed()) {
            State.mode = State.Modes.k_arm;
        } else if (driveController.getLeftBumperPressed() && driveController.getPOV() == 225) {
            State.mode = State.Modes.k_config;
        }
    }

    @Override
    public void changeState() {
        State.Drive.xSpeed = -driveController.getLeftY();
        State.Drive.zRotation = -driveController.getRightX();
        State.Drive.state = State.Drive.States.s_fastDrive;

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

        if (joystick.getRawButton(2)) {
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Arm.InitialHeight;
            State.Arm.targetDepth = Const.Arm.InitialDepth;
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
            State.Hand.rotateState = RotateState.s_turnHandBack;
        } else if (driveController.getXButton()) {
            switch (phase) {
                case Phase1:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
                    State.Hand.rotateState = RotateState.s_turnHandBack;
                    if (State.Arm.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase2;
                    }
                case Phase2:
                    State.Hand.grabHandState = GrabHandState.s_releaseHand;
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armIntakeHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armIntakeDepth;
                    if (State.Arm.isAtTarget()) {
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
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    break;
            }
        }

        if (driveController.getAButton()) {
            State.Drive.state = State.Drive.States.s_aprilTagTracking;
            State.cameraXSpeed = -driveController.getLeftY();
        } else if (driveController.getBButton()) {
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_limelightTracking;
            State.limelightXSpeed = -driveController.getLeftY();
        }

        if (driveController.getBButtonPressed()) {
            State.pidLimelightReset = true;
            State.Drive.state = State.Drive.States.s_limelightTracking;
        }

        if (driveController.getLeftBumper() && driveController.getRightBumper()) {
            State.intakeExtensionState = State.IntakeExtensionState.s_closeIntake;
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
