package frc.robot.mode;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.State;
import frc.robot.State.GrabHandState;
import frc.robot.State.MoveLeftAndRightArmState;
import frc.robot.State.RollerState;
import frc.robot.State.Hand.RotateState;
import frc.robot.subClass.Const;
import frc.robot.subClass.Tools;

public class DriveMode extends Mode {

    private static GrabGamePiecePhase phase = GrabGamePiecePhase.Phase1;
    private static int GrabCount = 0;

    @Override
    public void changeMode() {
        if (driveController.getBackButton()) {
            State.mode = State.Modes.k_arm;
        } else if (driveController.getLeftBumperPressed() && driveController.getPOV() == 0) {
            State.mode = State.Modes.k_chargeStation;
        } else if (driveController.getLeftBumperPressed() && driveController.getPOV() == 225) {
            State.mode = State.Modes.k_config;
        }
    }

    @Override
    public void changeState() {
        State.Drive.xSpeed = -1 * driveController.getLeftY();
        State.Drive.zRotation = -1 * driveController.getRightX();
        State.Drive.state = State.Drive.States.s_fastDrive;

        //RT: intake, LT: outtake
        if (driveController.getRightTriggerAxis() > 0.5) {
            State.intakeState = RollerState.s_intakeGamePiece;
        } else if (driveController.getLeftTriggerAxis() > 0.5) {
            State.intakeState = RollerState.s_outtakeGamePiece;
        } else {
            State.intakeState = RollerState.s_stopRoller;
        }

        //YボタンでBasicPositionに戻る, RTボタンでゲームピースをつかんでbasicPositionに戻る
        if (driveController.getRightBumperPressed()) {
            phase = GrabGamePiecePhase.Phase1;
        }

        if (joystick.getRawButton(2)) {
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Arm.InitialHeight;
            State.Arm.targetDepth = Const.Arm.InitialDepth;
            State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
            State.Hand.rotateState = RotateState.s_turnHandBack;
        } else if (driveController.getRightBumper()) {
            SmartDashboard.putString("intakePhase", phase.toString());
            switch (phase) {
                case Phase1:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    State.moveLeftAndRightArmState = MoveLeftAndRightArmState.s_movetomiddle;
                    State.Hand.rotateState = RotateState.s_turnHandBack;
                    State.Hand.grabHandState = GrabHandState.s_releaseHand;
                    if (State.Arm.isAtTarget()) {
                        State.Hand.targetAngle = State.Hand.actualHandAngle + 90;
                        phase = GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    State.Hand.grabHandState = GrabHandState.s_releaseHand;
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = ( Const.Arm.InitialHeight+Const.GrabGamePiecePhase.armIntakeHeight) / 2 +5;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armIntakeDepth;
                    State.Hand.rotateState = RotateState.s_moveHandToSpecifiedAngle;
                    if (State.Arm.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    State.Hand.grabHandState = GrabHandState.s_releaseHand;
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armIntakeHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armIntakeDepth;
                    if (State.Arm.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase4;
                    }
                    break;
                case Phase4:
                    State.Hand.grabHandState = GrabHandState.s_grabHand;
                    GrabCount++;
                    if (GrabCount >= 20) {
                        phase = GrabGamePiecePhase.Phase5;
                        GrabCount = 0;
                    }
                    break;
                case Phase5:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Hand.rotateState = RotateState.s_turnHandBack;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    break;
            }
        } else {
            if(joystick.getPOV() == 0) {
                ArmMode.adjustArmPosition(0, Const.Arm.TargetModifyRatio);
            } else if(joystick.getPOV() == 180) {
                ArmMode.adjustArmPosition(0,  -Const.Arm.TargetModifyRatio);
            } else  if(joystick.getPOV() == 90) {
                ArmMode.adjustArmPosition(Const.Arm.TargetModifyRatio, 0);
            } else if(joystick.getPOV() == 270) {
                ArmMode.adjustArmPosition(- Const.Arm.TargetModifyRatio, 0);
            } else  if(joystick.getPOV() == 45) {
                ArmMode.adjustArmPosition(Const.Arm.TargetModifyRatio, Const.Arm.TargetModifyRatio);
            } else if(joystick.getPOV() == 135) {
                ArmMode.adjustArmPosition(Const.Arm.TargetModifyRatio, -Const.Arm.TargetModifyRatio);
            }else  if(joystick.getPOV() == 225) {
                ArmMode.adjustArmPosition(-Const.Arm.TargetModifyRatio, -Const.Arm.TargetModifyRatio);
            } else if(joystick.getPOV() == 315) {
                ArmMode.adjustArmPosition(-Const.Arm.TargetModifyRatio, Const.Arm.TargetModifyRatio);
            }
        }

        if (driveController.getAButton()) {
            State.Drive.state = State.Drive.States.s_aprilTagTracking;
            State.cameraXSpeed = -driveController.getLeftY();
        }// else if (driveController.getBButton()) {
        //     State.Drive.state = State.Drive.States.s_limelightTracking;
        //     State.limelightXSpeed = -driveController.getLeftY();
        // }

        if (driveController.getBButtonPressed()) {
            State.pidLimelightReset = true;
            State.Drive.state = State.Drive.States.s_limelightTracking;
        }

        if (driveController.getLeftBumper() && driveController.getRightBumper()) {
            State.intakeExtensionState = State.IntakeExtensionState.s_closeIntake;
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

    // ToDo  嘘なので直す
    enum GrabGamePiecePhase {
        //basicPositionに移動する
        Phase1,
        //ハンドを開ける, アームを下げる
        Phase2,
        //ハンドを閉める
        Phase3,
        //アームを上げる
        Phase4,
        Phase5
    }

}
