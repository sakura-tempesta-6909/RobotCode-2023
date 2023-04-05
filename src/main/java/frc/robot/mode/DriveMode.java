package frc.robot.mode;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.*;
import frc.robot.consts.*;
import frc.robot.subClass.Tools;
import frc.robot.subClass.Util;

public class DriveMode extends Mode {

    private static GrabGamePiecePhase phase = GrabGamePiecePhase.Phase1;
    private static int GrabCount = 0;

    @Override
    public void changeMode() {
        if (driveController.getBackButton()) {
            State.mode = State.Modes.k_arm;
        } else if (driveController.getPOV() == 0) {
            State.mode = State.Modes.k_chargeStation;
        } else if (driveController.getLeftBumperPressed() && driveController.getPOV() == 225) {
            State.mode = State.Modes.k_config;
        }
    }

    @Override
    public void changeState() {
        DriveState.xSpeed = -1 * driveController.getLeftY();
        DriveState.zRotation = -1 * driveController.getRightX();
        if (driveController.getRightBumper()) {
            DriveState.driveState = DriveState.DriveStates.s_midDrive;
        } else {
            DriveState.driveState = DriveState.DriveStates.s_fastDrive;
        }

        //RT: intake, LT: outtake
        if (driveController.getRightTriggerAxis() > 0.5) {
            IntakeState.intakeState = IntakeState.RollerStates.s_intakeGamePiece;
            ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;

            // Intake時に引っかからない位置に移動
            ArmState.targetHeight = IntakeConst.armHeight;
            ArmState.targetDepth = IntakeConst.armDepth;
        } else if (driveController.getLeftTriggerAxis() > 0.5) {
            IntakeState.intakeState = IntakeState.RollerStates.s_outtakeGamePiece;
            ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;

            // Intake時に引っかからない位置に移動
            ArmState.targetHeight = IntakeConst.armHeight;
            ArmState.targetDepth = IntakeConst.armDepth;
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

        if (joystick.getRawButtonPressed(11) || joystick.getRawButtonPressed(12) || joystick.getRawButtonPressed(10) ||  joystick.getRawButtonPressed(7)) {
            phase = GrabGamePiecePhase.Phase1;
        }

        if (joystick.getRawButtonPressed(10)) {
            DriveState.resetPosition = true;
            DriveState.resetPIDController = true;
        }

        if (joystick.getRawButton(2)) {
            ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
            Util.Calculate.setInitWithRelay();
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
            HandState.rotateState = HandState.RotateStates.s_turnHandBack;
        } else if (joystick.getRawButton(12)) {
            // キューブ
            SmartDashboard.putString("intakePhase", phase.toString());
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

                    if (ArmState.isAtTarget()) {
                        // 次の段階で初期位置から90度回すようにする
                        HandState.targetAngle = HandState.actualHandAngle + 90;
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
        } else if (joystick.getRawButton(11)) {
            // コーン
            SmartDashboard.putString("intakePhase", phase.toString());
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
                    ArmState.targetHeight = GrabGamePiecePhaseConst.armConePrepareHeight;
                    ArmState.targetDepth = GrabGamePiecePhaseConst.armConePrepareDepth;

                    if (ArmState.isAtTarget() && HandState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    // アームを下げる
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    // 左右はど真ん中にする
                    ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;

                    // ハンドを開く
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;

                    // コーンを掴む位置
                    ArmState.targetHeight = GrabGamePiecePhaseConst.armConeGrabHeight;
                    ArmState.targetDepth = GrabGamePiecePhaseConst.armConeGrabDepth;

                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    // 左右はど真ん中にする
                    ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;

                    // ハンドを止める
                    HandState.rotateState = HandState.RotateStates.s_stopHand;
                    // コーンを掴む！！！
                    HandState.grabHandState = HandState.GrabHandStates.s_grabHand;

                    GrabCount++;
                    if (GrabCount >= 20) {
                        phase = GrabGamePiecePhase.Phase4;
                        GrabCount = 0;
                    }
                    break;
                case Phase4:
                    // アームをBasic(Initial)Positionに戻す
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;

                    // ハンドを初期位置に戻す
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;

                    // BasicPositionにターゲットを設定
                    Util.Calculate.setInitWithRelay();
                    break;
            }   
        }else if (joystick.getRawButton(10)) {
            // サブステーション
            SmartDashboard.putString("substationPhase", phase.toString());
            switch (phase){
                case Phase1:
                    // PIDでちょっと下がる
                    DriveState.driveState = DriveState.DriveStates.s_pidDrive;

                    // PIDでどんくらい下がるか
                    DriveState.targetMeter = -1;

                    if (DriveState.isAtTarget()){
                        phase = GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    // アームを初期位置に
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    // 左右はど真ん中にする
                    ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;

                    // ハンドを初期位置に戻す
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;

                    // アームの初期位置を設定
                    Util.Calculate.setInitWithRelay();
                    if (ArmState.isAtTarget() && HandState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    // アームをリレーポイントへ
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;

                    // リレーポイント
                    ArmState.targetHeight = ArmConst.RelayPointToGoalHeight;
                    ArmState.targetDepth = ArmConst.RelayPointToGoalDepth;

                    if (Util.Calculate.isOverRelayToGoal(ArmState.actualHeight, ArmState.actualDepth)) {
                        phase = GrabGamePiecePhase.Phase4;
                    }
                    break;
                case Phase4:
                    // アームをサブステーションの位置へ
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;

                    // ハンドを開く
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;

                    // サブステーションの位置
                    ArmState.targetHeight = GrabGamePiecePhaseConst.armSubStationHeight;
                    ArmState.targetDepth = GrabGamePiecePhaseConst.armSubStationDepth;
                    break;
            }
        } else
            if(joystick.getPOV() == 0) {
                ArmMode.adjustArmPosition(0, ArmConst.TargetModifyRatio);
            } else if(joystick.getPOV() == 180) {
                ArmMode.adjustArmPosition(0,  -ArmConst.TargetModifyRatio);
            } else  if(joystick.getPOV() == 90) {
                ArmMode.adjustArmPosition(ArmConst.TargetModifyRatio, 0);
            } else if(joystick.getPOV() == 270) {
                ArmMode.adjustArmPosition(- ArmConst.TargetModifyRatio, 0);
            } else  if(joystick.getPOV() == 45) {
                ArmMode.adjustArmPosition(ArmConst.TargetModifyRatio, ArmConst.TargetModifyRatio);
            } else if(joystick.getPOV() == 135) {
                ArmMode.adjustArmPosition(ArmConst.TargetModifyRatio, -ArmConst.TargetModifyRatio);
            }else  if(joystick.getPOV() == 225) {
                ArmMode.adjustArmPosition(-ArmConst.TargetModifyRatio, -ArmConst.TargetModifyRatio);
            } else if(joystick.getPOV() == 315) {
                ArmMode.adjustArmPosition(-ArmConst.TargetModifyRatio, ArmConst.TargetModifyRatio);
            }
        

        if (driveController.getAButton()) {
            DriveState.driveState = DriveState.DriveStates.s_aprilTagTracking;
            CameraState.cameraXSpeed = -driveController.getLeftY();
        } else if (driveController.getBButton()) {
            LimelightState.isLimelightOn = true;
            DriveState.driveState = DriveState.DriveStates.s_limelightTracking;
            LimelightState.limelightXSpeed = -driveController.getLeftY();
        }

        if (driveController.getBButtonPressed()) {
            LimelightState.pidLimelightReset = true;
            DriveState.driveState = DriveState.DriveStates.s_limelightTracking;
        }

        if (driveController.getLeftBumper() && driveController.getRightBumper()) {
            IntakeState.intakeExtensionState = IntakeState.IntakeExtensionStates.s_closeIntake;
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
