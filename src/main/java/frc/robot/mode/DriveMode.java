package frc.robot.mode;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.*;
import frc.robot.subClass.Const;
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

        if (joystick.getRawButtonPressed(11) || joystick.getRawButtonPressed(12) || joystick.getRawButtonPressed(10) ||  joystick.getRawButtonPressed(7)) {
            phase = GrabGamePiecePhase.Phase1;
        }

        if (joystick.getRawButtonPressed(10)) {
            DriveState.resetPosition = true;
            DriveState.resetPIDController = true;
        }

        if (joystick.getRawButton(2)) {
            ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
            ArmState.targetHeight = Const.Arm.InitialHeight;
            ArmState.targetDepth = Const.Arm.InitialDepth;
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
            HandState.rotateState = HandState.RotateStates.s_turnHandBack;
        } else if (joystick.getRawButton(12)) {
            // キューブ
            SmartDashboard.putString("intakePhase", phase.toString());
            switch (phase) {
                case Phase1:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = Const.Arm.InitialHeight;
                    ArmState.targetDepth = Const.Arm.InitialDepth;
                    ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                    if (ArmState.isAtTarget()) {
                        HandState.targetAngle = HandState.actualHandAngle + 90;
                        phase = GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = ( Const.Arm.InitialHeight+Const.GrabGamePiecePhase.armCubeIntakeHeight) / 2 +5;
                    ArmState.targetDepth = Const.GrabGamePiecePhase.armCubeIntakeDepth;
                    HandState.rotateState = HandState.RotateStates.s_moveHandToSpecifiedAngle;
                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = Const.GrabGamePiecePhase.armCubeIntakeHeight;
                    ArmState.targetDepth = Const.GrabGamePiecePhase.armCubeIntakeDepth;
                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase4;
                    }
                    break;
                case Phase4:
                    HandState.grabHandState = HandState.GrabHandStates.s_grabHand;
                    GrabCount++;
                    if (GrabCount >= 20) {
                        phase = GrabGamePiecePhase.Phase5;
                        GrabCount = 0;
                    }
                    break;
                case Phase5:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                    ArmState.targetHeight = Const.Arm.InitialHeight;
                    ArmState.targetDepth = Const.Arm.InitialDepth;
                    break;
            }
        } else if (joystick.getRawButton(11)) {
            // コーン
            SmartDashboard.putString("intakePhase", phase.toString());
            switch (phase) {
                case Phase1:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = Const.Arm.InitialHeight;
                    ArmState.targetDepth = Const.Arm.InitialDepth;
                    // ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                    HandState.grabHandState = HandState.GrabHandStates.s_grabHand;
                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    HandState.grabHandState = HandState.GrabHandStates.s_grabHand;
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = (Const.GrabGamePiecePhase.armConeIntakeHeight + Const.Arm.InitialHeight) / 2;
                    ArmState.targetDepth = Const.GrabGamePiecePhase.armConeIntakeDepth;
                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    HandState.grabHandState = HandState.GrabHandStates.s_grabHand;
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = Const.GrabGamePiecePhase.armConeIntakeRelesaseHeight;
                    ArmState.targetDepth = Const.GrabGamePiecePhase.armConeIntakeDepth;
                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase4;
                    }
                    break;
                case Phase4:
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = Const.GrabGamePiecePhase.armConeIntakeRelesaseHeight;
                    ArmState.targetDepth = Const.GrabGamePiecePhase.armConeIntakeDepth;
                    GrabCount++;
                    if (GrabCount >= 80) {
                        phase = GrabGamePiecePhase.Phase5;
                        GrabCount = 0;
                    }
                    break;
                case Phase5:
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = Const.GrabGamePiecePhase.armConeIntakeHeight;
                    ArmState.targetDepth = Const.GrabGamePiecePhase.armConeIntakeDepth;
                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase6;
                    }
                    break;
                case Phase6:
                    HandState.grabHandState = HandState.GrabHandStates.s_grabHand;
                    GrabCount++;
                    if (GrabCount >= 20) {
                        phase = GrabGamePiecePhase.Phase7;
                        GrabCount = 0;
                    }
                    break;
                case Phase7:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                    ArmState.targetHeight = Const.Arm.InitialHeight;
                    ArmState.targetDepth = Const.GrabGamePiecePhase.armConeIntakeDepth;
                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase8;
                    }
                    break;
                case Phase8:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                    ArmState.targetHeight = Const.Arm.InitialHeight;
                    ArmState.targetDepth = Const.Arm.InitialDepth;
                    break;
            }   
        }else if (joystick.getRawButton(10)) {
            switch (phase){
                case Phase1:
                    DriveState.targetMeter = -1;
                    DriveState.driveState = DriveState.DriveStates.s_pidDrive;
                    if (DriveState.isAtTarget() || true){
                        phase = GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = Const.Arm.InitialHeight;
                    ArmState.targetDepth = Const.Arm.InitialDepth;
                    ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                    HandState.grabHandState = HandState.GrabHandStates.s_grabHand;
                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = Const.GrabGamePiecePhase.armRelayPointHeight;
                    ArmState.targetDepth = Const.GrabGamePiecePhase.armRelayPointDepth;
                    if (Util.Calculate.relayReach(ArmState.actualHeight, ArmState.actualDepth)) {
                        phase = GrabGamePiecePhase.Phase4;
                    }
                    break;
                case Phase4:
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = Const.GrabGamePiecePhase.armSubStationHeight;
                    ArmState.targetDepth = Const.GrabGamePiecePhase.armSubStationDepth;
                    break;
            }
        } else if(joystick.getRawButton(7)) {
            switch (phase) {
                case Phase1:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = Const.Arm.InitialHeight;
                    ArmState.targetDepth = Const.Arm.InitialDepth;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                    if (ArmState.isAtTarget()) {
                        HandState.targetAngle = HandState.actualHandAngle + 90;
                        phase = GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = ( Const.Arm.InitialHeight+Const.GrabGamePiecePhase.armCubeIntakeHeight) / 2 +5;
                    ArmState.targetDepth = Const.GrabGamePiecePhase.armCubeIntakeDepth;
                    HandState.rotateState = HandState.RotateStates.s_moveHandToSpecifiedAngle;
                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    ArmState.targetHeight = Const.GrabGamePiecePhase.armConeIntakeHeight;
                    ArmState.targetDepth = Const.GrabGamePiecePhase.armCubeIntakeDepth;
                    if (ArmState.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase4;
                    }
                    break;
                case Phase4:
                    HandState.grabHandState = HandState.GrabHandStates.s_grabHand;
                    GrabCount++;
                    if (GrabCount >= 20) {
                        phase = GrabGamePiecePhase.Phase5;
                        GrabCount = 0;
                    }
                    break;
                case Phase5:
                    ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
                    HandState.rotateState = HandState.RotateStates.s_turnHandBack;
                    ArmState.targetHeight = Const.Arm.InitialHeight;
                    ArmState.targetDepth = Const.Arm.InitialDepth;
                    break;
            }
       } else 
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
