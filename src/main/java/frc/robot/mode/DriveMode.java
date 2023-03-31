package frc.robot.mode;

import java.util.Map;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.States.LimelightState;
import frc.robot.States.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.Tools;

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
        State.Drive.xSpeed = -1 * driveController.getLeftY();
        State.Drive.zRotation = -1 * driveController.getRightX();
        State.Drive.state = State.Drive.States.s_fastDrive;

        //RT: intake, LT: outtake
        if (driveController.getRightTriggerAxis() > 0.5) {
            State.intakeState = State.RollerState.s_intakeGamePiece;
        } else if (driveController.getLeftTriggerAxis() > 0.5) {
            State.intakeState = State.RollerState.s_outtakeGamePiece;
        } else {
            State.intakeState = State.RollerState.s_stopRoller;
        }

        if (joystick.getRawButton(1)) {
            // ハンドを開く
            State.Hand.grabHandState = State.GrabHandState.s_releaseHand;
        }

        if (joystick.getRawButton(3)) {
            // 手首の位置をリセット
            State.Hand.rotateState = State.Hand.RotateState.s_turnHandBack;
        } else if (joystick.getRawButton(5)) {
            // 手首が右回転する
            State.Hand.rotateState = State.Hand.RotateState.s_rightRotateHand;
        } else if (joystick.getRawButton(6)) {
            // 手首が左回転する
            State.Hand.rotateState = State.Hand.RotateState.s_leftRotateHand;
        } else if (joystick.getRawButton(4)) {
            // 手首が180°回転する
            State.Hand.rotateState = State.Hand.RotateState.s_moveHandToSpecifiedAngle;
        }
        if (joystick.getRawButtonPressed(4)) {
            // 手首が180°回転する
            State.Hand.targetAngle = State.Hand.actualHandAngle + 180;
            State.Hand.isResetHandPID = true;
        }

        if (joystick.getRawButtonPressed(11) || joystick.getRawButtonPressed(12) || joystick.getRawButtonPressed(10)) {
            phase = GrabGamePiecePhase.Phase1;
        }

        if (joystick.getRawButtonPressed(10)) {
            State.Drive.resetPosition = true;
            State.Drive.resetPIDController = true;
        }

        if (joystick.getRawButton(2)) {
            State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
            State.Arm.targetHeight = Const.Arm.InitialHeight;
            State.Arm.targetDepth = Const.Arm.InitialDepth;
            State.moveLeftAndRightArmState = State.MoveLeftAndRightArmState.s_movetomiddle;
            State.Hand.rotateState = State.Hand.RotateState.s_turnHandBack;
        } else if (joystick.getRawButton(12)) {
            // キューブ
            SmartDashboard.putString("intakePhase", phase.toString());
            switch (phase) {
                case Phase1:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    State.moveLeftAndRightArmState = State.MoveLeftAndRightArmState.s_movetomiddle;
                    State.Hand.rotateState = State.Hand.RotateState.s_turnHandBack;
                    State.Hand.grabHandState = State.GrabHandState.s_releaseHand;
                    if (State.Arm.isAtTarget()) {
                        State.Hand.targetAngle = State.Hand.actualHandAngle + 90;
                        phase = GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    State.Hand.grabHandState = State.GrabHandState.s_releaseHand;
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = ( Const.Arm.InitialHeight+Const.GrabGamePiecePhase.armCubeIntakeHeight) / 2 +5;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armCubeIntakeDepth;
                    State.Hand.rotateState = State.Hand.RotateState.s_moveHandToSpecifiedAngle;
                    if (State.Arm.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    State.Hand.grabHandState = State.GrabHandState.s_releaseHand;
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armCubeIntakeHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armCubeIntakeDepth;
                    if (State.Arm.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase4;
                    }
                    break;
                case Phase4:
                    State.Hand.grabHandState = State.GrabHandState.s_grabHand;
                    GrabCount++;
                    if (GrabCount >= 20) {
                        phase = GrabGamePiecePhase.Phase5;
                        GrabCount = 0;
                    }
                    break;
                case Phase5:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Hand.rotateState = State.Hand.RotateState.s_turnHandBack;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    break;
            }
        } else if (joystick.getRawButton(11)) {
            // コーン
            SmartDashboard.putString("intakePhase", phase.toString());
            switch (phase) {
                case Phase1:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    State.moveLeftAndRightArmState = State.MoveLeftAndRightArmState.s_movetomiddle;
                    State.Hand.rotateState = State.Hand.RotateState.s_turnHandBack;
                    State.Hand.grabHandState = State.GrabHandState.s_grabHand;
                    if (State.Arm.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    State.Hand.grabHandState = State.GrabHandState.s_grabHand;
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = (Const.GrabGamePiecePhase.armConeIntakeHeight + Const.Arm.InitialHeight) / 2;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armConeIntakeDepth;
                    if (State.Arm.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    State.Hand.grabHandState = State.GrabHandState.s_grabHand;
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armConeIntakeRelesaseHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armConeIntakeDepth;
                    if (State.Arm.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase4;
                    }
                    break;
                case Phase4:
                    State.Hand.grabHandState = State.GrabHandState.s_releaseHand;
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armConeIntakeRelesaseHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armConeIntakeDepth;
                    GrabCount++;
                    if (GrabCount >= 80) {
                        phase = GrabGamePiecePhase.Phase5;
                        GrabCount = 0;
                    }
                    break;
                case Phase5:
                    State.Hand.grabHandState = State.GrabHandState.s_releaseHand;
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armConeIntakeHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armConeIntakeDepth;
                    if (State.Arm.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase6;
                    }
                    break;
                case Phase6:
                    State.Hand.grabHandState = State.GrabHandState.s_grabHand;
                    GrabCount++;
                    if (GrabCount >= 20) {
                        phase = GrabGamePiecePhase.Phase7;
                        GrabCount = 0;
                    }
                    break;
                case Phase7:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Hand.rotateState = State.Hand.RotateState.s_turnHandBack;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armConeIntakeDepth;
                    if (State.Arm.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase8;
                    }
                    break;
                case Phase8:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Hand.rotateState = State.Hand.RotateState.s_turnHandBack;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    break;
            }
        }else if (joystick.getRawButton(10)) {
            switch (phase){
                case Phase1:
                    State.Drive.targetMeter = -1;
                    State.Drive.state = State.Drive.States.s_pidDrive;
                    if (State.Drive.isAtTarget() || true){
                        phase = GrabGamePiecePhase.Phase2;
                    }
                    break;
                case Phase2:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.Arm.InitialHeight;
                    State.Arm.targetDepth = Const.Arm.InitialDepth;
                    State.moveLeftAndRightArmState = State.MoveLeftAndRightArmState.s_movetomiddle;
                    State.Hand.rotateState = State.Hand.RotateState.s_turnHandBack;
                    State.Hand.grabHandState = State.GrabHandState.s_grabHand;
                    if (State.Arm.isAtTarget()) {
                        phase = GrabGamePiecePhase.Phase3;
                    }
                    break;
                case Phase3:
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armRelayPointHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armRelayPointDepth;
                    State.Hand.rotateState = State.Hand.RotateState.s_moveHandToSpecifiedAngle;
                    if (State.Arm.actualHeight > -10) {
                        phase = GrabGamePiecePhase.Phase4;
                    }
                    break;
                case Phase4:
                    State.Hand.grabHandState = State.GrabHandState.s_releaseHand;
                    State.Arm.state = State.Arm.States.s_moveArmToSpecifiedPosition;
                    State.Hand.rotateState = State.Hand.RotateState.s_moveHandToSpecifiedAngle;
                    State.Arm.targetHeight = Const.GrabGamePiecePhase.armSubStationHeight;
                    State.Arm.targetDepth = Const.GrabGamePiecePhase.armSubStationDepth;
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
        } else if (driveController.getBButton()) {
            LimelightState.isLimelightOn = true;
            State.Drive.state = State.Drive.States.s_limelightTracking;
            State.limelightXSpeed = -driveController.getLeftY();
        }

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
        Phase5,
        Phase6,
        Phase7,
        Phase8,
    }

}
