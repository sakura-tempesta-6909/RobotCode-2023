package frc.robot.mode;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.*;
import frc.robot.states.HandState.GrabHandStates;
import frc.robot.states.IntakeState.IntakeExtensionStates;
import frc.robot.states.LimelightState.States;
import frc.robot.consts.*;
import frc.robot.mode.ChargeStationMode.GrabGamePiecePhase;
import frc.robot.subClass.Tools;
import frc.robot.subClass.Util;

public class DriveMode extends Mode {

    private static GrabGamePiecePhase phase = GrabGamePiecePhase.Phase1;
    private static int GrabCount = 0;
    private static double limelightTotalDistance;
    private static double limelightDitectionCount;
    private static double limelightAveraveDistance;

    private static boolean isBasicRelayOver;

    @Override
    public void changeMode() {
        if (driveController.getBackButton()) {
            State.mode = State.Modes.k_arm;
            IntakeState.intakeExtensionState = IntakeExtensionStates.s_closeIntake;
        } else if (driveController.getPOV() == 0) {
            State.mode = State.Modes.k_chargeStation;
        }
    }

    @Override
    public void changeState() {
        DriveState.xSpeed = -1 * driveController.getLeftY();
        DriveState.zRotation = -1 * driveController.getRightX();
        if (driveController.getRightBumper()) {
            DriveState.driveState = DriveState.DriveStates.s_slowDrive;
        } else {
            DriveState.driveState = DriveState.DriveStates.s_fastDrive;
        }

        if (driveController.getAButtonPressed()) {
            if (IntakeState.intakeExtensionState == IntakeState.IntakeExtensionStates.s_closeIntake) {
                IntakeState.intakeExtensionState = IntakeExtensionStates.s_openIntake;
            } else {
                IntakeState.intakeExtensionState = IntakeExtensionStates.s_closeIntake;
            }

        }
        if (driveController.getXButtonPressed()) {
            if (!LimelightState.isLimelightFlashing) {
                LimelightState.isLimelightFlashing = true;
            } else {
                LimelightState.isLimelightFlashing = false;
            }
        }

        // if (IntakeState.intakeExtensionState == IntakeExtensionStates.s_closeIntake) {
        //     IntakeState.intakeState = IntakeState.RollerStates.s_stopRoller;
        // }

        //RT: intake, LT: outtake
        if (driveController.getRightTriggerAxis() > 0.5) {
            IntakeState.intakeState = IntakeState.RollerStates.s_intakeGamePiece;
        } else if (driveController.getLeftTriggerAxis() > 0.5) {
            IntakeState.intakeState = IntakeState.RollerStates.s_outtakeGamePiece;
        } else {
            IntakeState.intakeState = IntakeState.RollerStates.s_stopRoller;
        }

        final double joystickZ = 1 * Tools.deadZoneProcess(joystick.getRawAxis(2));


        if (driveController.getRightBumper() && driveController.getLeftBumper()) {
            // アームの位置をリセット
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
        } else if (joystickZ > 0.5) {
            // アームを右に動かす
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_moveRightMotor;
        } else if (joystickZ < -0.5) {
            // アームを左に動かす
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_moveLeftMotor;
        }

        if (joystick.getRawButton(1)) {
            // ハンドを開く
            HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
        }

        if (joystick.getRawButton(3)) {
            // 手首の位置をリセット
            HandState.rotateState = HandState.RotateStates.s_turnHandBack;
        } else if (joystick.getRawButton(5)) {
            // 手首が右回転する
            HandState.rotateState = HandState.RotateStates.s_rightRotateHand;
        } else if (joystick.getRawButton(6)) {
            // 手首が左回転する
            HandState.rotateState = HandState.RotateStates.s_leftRotateHand;
        } else if (joystick.getRawButton(4)) {
            // 手首が180°回転する
            HandState.rotateState = HandState.RotateStates.s_moveHandToSpecifiedAngle;
        }
        if (joystick.getRawButtonPressed(4)) {
            // 手首が180°回転する
            HandState.targetAngle = HandState.actualHandAngle + 180;
            HandState.isResetHandPID = true;
        }

        if (joystick.getRawButtonPressed(11) || joystick.getRawButtonPressed(12) || joystick.getRawButtonPressed(10) || joystick.getRawButtonPressed(7) || joystick.getRawButtonPressed(9) || joystick.getRawButtonPressed(2)) {
            phase = GrabGamePiecePhase.Phase1;
            DriveState.resetPosition = true;
            DriveState.resetPIDController = true;
            ArmState.firstRelayToIntakeOver = false;
            ArmState.secondRelayToIntakeOver = false;
            GrabCount = 0;
            limelightDitectionCount = 0;
            limelightTotalDistance = 0;
            isBasicRelayOver = false;
        }

        isBasicRelayOver |= ArmState.actualDepth < 30;
        SmartDashboard.putBoolean("BasicRelayOver", isBasicRelayOver);
        if (joystick.getRawButton(2)) {
            Util.Calculate.setIntakeWithRelay();
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
            HandState.rotateState = HandState.RotateStates.s_turnHandBack;
        } else if (joystick.getRawButton(12)) {
            // キューブ
            SmartDashboard.putString("intakePhase", phase.toString());
            if (IntakeState.intakeExtensionState == IntakeExtensionStates.s_openIntake) {
                switch (phase) {
                    case Phase1:
                        // アームを準備段階の高さまで動かす
                        ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                        // 左右はど真ん中にする
                        ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;

                        // ハンドを初期位置に回す
                        HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                        // ハンドを開く
                        HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;

                        // 準備段階の位置
                        ArmState.targetHeight = GrabGamePiecePhaseConst.armCubePrepareHeight;
                        ArmState.targetDepth = GrabGamePiecePhaseConst.armCubePrepareDepth;

                        if (ArmState.isAtTarget() && HandState.isAtTarget()) {
                            // 次の段階で初期位置から90度回すようにする
                            HandState.targetAngle = HandState.actualHandAngle + 90;
                            HandState.rotateState = HandState.RotateStates.s_stopHand;
                            phase = GrabGamePiecePhase.Phase2;
                        }
                        break;
                    case Phase2:
                        // ハンドを開く
                        HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                        // ハンドを90度に回転する
                        HandState.rotateState = HandState.RotateStates.s_moveHandToSpecifiedAngle;

                        if (HandState.isAtTarget()) {
                            phase = GrabGamePiecePhase.Phase3;
                        }
                        break;
                    case Phase3:
                        // アームを掴むところまでおろす
                        ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;

                        // ハンドを開く
                        HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;

                        // アームをおろして掴む位置
                        ArmState.targetHeight = GrabGamePiecePhaseConst.armCubeGrabHeight;
                        ArmState.targetDepth = GrabGamePiecePhaseConst.armCubeGrabDepth;

                        if (ArmState.isAtTarget()) {
                            phase = GrabGamePiecePhase.Phase4;
                        }
                        break;
                    case Phase4:
                        // キューブを掴む！！
                        HandState.grabHandState = HandState.GrabHandStates.s_grabHand;

                        GrabCount++;
                        if (GrabCount >= 20) {
                            phase = GrabGamePiecePhase.Phase5;
                            GrabCount = 0;
                        }
                        break;
                    case Phase5:
                        // アームをBasic(Initial)Positionに戻す
                        ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;

                        // ハンドを初期位置に戻す
                        HandState.rotateState = HandState.RotateStates.s_turnHandBack;

                        // BasicPositionにターゲットを設定
                        Util.Calculate.setInitWithRelay();
                        break;

                }
            }

        } else if (joystick.getRawButton(11)) {
            // コーン
            SmartDashboard.putString("intakePhase", phase.toString());
            if (IntakeState.intakeExtensionState == IntakeExtensionStates.s_openIntake) {
                switch (phase) {
                    case Phase1:
                        // アームを準備段階の高さまで動かす
                        ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                        // 左右はど真ん中にする
                        // ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;

                        // ハンドを初期位置に回す
                        HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                        // ハンドを開く
                        HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;

                        // 準備段階の位置
                        ArmState.targetHeight = GrabGamePiecePhaseConst.armConePrepareHeight;
                        ArmState.targetDepth = GrabGamePiecePhaseConst.armConePrepareDepth;

                        GrabCount++;
                        if (ArmState.isAtTarget() && HandState.isAtTarget() && GrabCount >= 40) {
                            phase = GrabGamePiecePhase.Phase2;
                            GrabCount = 0;
                        }
                        break;
                    case Phase2:
                        HandState.grabHandState = GrabHandStates.s_releaseHand;
                        GrabCount++;
                        if(true) {
                            phase = GrabGamePiecePhase.Phase3;
                            GrabCount = 0;
                        }
                        break;
                    case Phase3:
                        // アームを下げる
                        ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                        // 左右はど真ん中にする
                        // ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;

                        // ハンドを開く
                        HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;

                        // コーンを掴む位置
                        ArmState.targetHeight = GrabGamePiecePhaseConst.armConeGrabHeight;
                        ArmState.targetDepth = GrabGamePiecePhaseConst.armConeGrabDepth;

                        if (ArmState.isAtTarget()) {
                            phase = GrabGamePiecePhase.Phase4;
                        }
                        break;
                    case Phase4:
                        // 左右はど真ん中にする
                        // ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;

                        // コーンを掴む！！！
                        HandState.grabHandState = HandState.GrabHandStates.s_grabHand;

                        GrabCount++;
                        if (GrabCount >= 20) {
                            phase = GrabGamePiecePhase.Phase5;
                            GrabCount = 0;
                        }
                        break;
                    case Phase5:
                        // アームをBasic(Initial)Positionに戻す
                        ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;

                        // ハンドを初期位置に戻す
                        HandState.rotateState = HandState.RotateStates.s_turnHandBack;

                        // BasicPositionのちょっと上にターゲットを設定
                        ArmState.targetHeight = ArmConst.InitialHeight + 10;
                        ArmState.targetDepth = ArmConst.InitialDepth;
                        break;
                }
            }
        } else if (joystick.getRawButton(9)) {
            // サブステーションからコーンを取る
            SmartDashboard.putString("substationPhase", phase.toString());
            switch (phase) {
                case Phase1:
                    LimelightState.limelightState = LimelightState.States.s_coneDetection;
                    // アームをリレーポイントへ
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;

                    // リレーポイント
                    ArmState.targetHeight = ArmConst.RelayPointToGoalHeight;
                    ArmState.targetDepth = ArmConst.RelayPointToGoalDepth;

                    ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_limelightTracking;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;

                    if (LimelightState.tv) {
                        if (60 < LimelightState.armToCone && LimelightState.armToCone < 93) {
                            limelightDitectionCount += 1;
                            limelightTotalDistance += LimelightState.armToCone;
                            limelightAveraveDistance = limelightTotalDistance / limelightDitectionCount;
                        }

                    }
                    if (Util.Calculate.isOverRelayToGoal(ArmState.actualHeight, ArmState.actualDepth) && HandState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase2;
                        if (limelightDitectionCount == 0) {
                            ArmState.targetDepth = GrabGamePiecePhaseConst.armSubStationDepth;
                        } else {
                            ArmState.targetHeight = GrabGamePiecePhaseConst.armSubStationHeight;
                            if(65 >= limelightAveraveDistance) {
                                ArmState.targetDepth = limelightAveraveDistance;
                            }else {
                                ArmState.targetDepth = limelightAveraveDistance + 32 - 5;
                            }
                        }

                    }
                    break;
                case Phase2:
                    ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_limelightTracking;
                    // アームをサブステーションの位置へ
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;


                    // サブステーションの位置

                    break;
            }

        } else if (joystick.getRawButton(10)) {
            // サブステーションからキューブを取る
            SmartDashboard.putString("substationPhase", phase.toString());

            switch (phase) {
                case Phase1:
                    LimelightState.limelightState = LimelightState.States.s_cubeDetection;
                    // アームをリレーポイントへ
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;


                    // リレーポイント
                    ArmState.targetHeight = ArmConst.RelayPointToGoalHeight;
                    ArmState.targetDepth = ArmConst.RelayPointToGoalDepth;

                    ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_limelightTracking;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                    if (LimelightState.tv) {
                        if (50 < LimelightState.armToCube && LimelightState.armToCube < 75) {
                            limelightDitectionCount += 1;
                            limelightTotalDistance += LimelightState.armToCube;
                            limelightAveraveDistance = limelightTotalDistance / limelightDitectionCount;
                        }

                    }
                    if (Util.Calculate.isOverRelayToGoal(ArmState.actualHeight, ArmState.actualDepth) && HandState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase2;
                        if (limelightDitectionCount == 0) {
                            ArmState.targetDepth = GrabGamePiecePhaseConst.armSubStationDepth;
                        } else {
                            ArmState.targetHeight = GrabGamePiecePhaseConst.armSubStationHeight;
                            if(70 >= limelightAveraveDistance) {
                                ArmState.targetDepth = limelightAveraveDistance + 35;
                            }else {
                                ArmState.targetDepth = limelightAveraveDistance + 45;
                            }
                        }

                    }
                    break;
                case Phase2:
                    LimelightState.limelightState = States.s_cubeDetection;
                    ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_limelightTracking;
                    // アームをサブステーションの位置へ
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;

                    // サブステーションの位置

                    break;
                }


        } else if (joystick.getPOV() == 0) {
            ArmMode.adjustArmPosition(0, ArmConst.TargetModifyRatio);
        } else if (joystick.getPOV() == 180) {
            ArmMode.adjustArmPosition(0, -ArmConst.TargetModifyRatio);
        } else if (joystick.getPOV() == 90) {
            ArmMode.adjustArmPosition(ArmConst.TargetModifyRatio, 0);
        } else if (joystick.getPOV() == 270) {
            ArmMode.adjustArmPosition(-ArmConst.TargetModifyRatio, 0);
        } else if (joystick.getPOV() == 45) {
            ArmMode.adjustArmPosition(ArmConst.TargetModifyRatio, ArmConst.TargetModifyRatio);
        } else if (joystick.getPOV() == 135) {
            ArmMode.adjustArmPosition(ArmConst.TargetModifyRatio, -ArmConst.TargetModifyRatio);
        } else if (joystick.getPOV() == 225) {
            ArmMode.adjustArmPosition(-ArmConst.TargetModifyRatio, -ArmConst.TargetModifyRatio);
        } else if (joystick.getPOV() == 315) {
            ArmMode.adjustArmPosition(-ArmConst.TargetModifyRatio, ArmConst.TargetModifyRatio);
        }


        if (driveController.getBButton()) {
            LimelightState.isLimelightOn = true;
            DriveState.driveState = DriveState.DriveStates.s_limelightTracking;
            LimelightState.limelightXSpeed = -driveController.getLeftY();
        }

        if (driveController.getBButtonPressed()) {
            LimelightState.pidLimelightReset = true;
            DriveState.driveState = DriveState.DriveStates.s_limelightTracking;
        }

        // ターゲット座標からターゲットの角度を計算する
        Map<String, Double> targetAngles = Tools.calculateAngles(ArmState.targetDepth, ArmState.targetHeight);
        Double target = targetAngles.get("RootAngle");
        if (target != null) {
            ArmState.targetRootAngle = target;
        } else {
            ArmState.targetRootAngle = ArmState.actualRootAngle;
        }
        target = targetAngles.get("JointAngle");
        if (target != null) {
            ArmState.targetJointAngle = target;
        } else {
            ArmState.targetJointAngle = ArmState.actualJointAngle;
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
        Phase5,
        Phase6,
        Phase7,
        Phase8,
    }

}
