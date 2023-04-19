package frc.robot.mode;

import frc.robot.states.*;
import frc.robot.states.IntakeState.IntakeExtensionStates;
import frc.robot.states.LimelightState.States;
import frc.robot.consts.ArmConst;
import frc.robot.consts.CameraConst;
import frc.robot.consts.LimelightConst;
import frc.robot.subClass.Tools;
import frc.robot.subClass.Util;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmMode extends Mode {
    private static DriveMode.GrabGamePiecePhase phase = DriveMode.GrabGamePiecePhase.Phase1;
    private static boolean isBasicRelayOver;

    /**
     * StartButton - DriveModeに
     */
    @Override
    public void changeMode() {
        if (driveController.getStartButton()) {
            State.mode = State.Modes.k_drive;
        } else if (driveController.getPOV() == 0) {
            State.mode = State.Modes.k_chargeStation;
        }
    }

    /**
     * ** [ ]内の数字は優先順位<br><br>
     * Button 1 - ハンドを開く[0]<br>
     * Button 2 - すべてリセット　BasicPositionに戻る[0]<br><br>
     * Button 3 - 手首の位置をリセット[4]<br>
     * Button 4 - 手首を180°回転する[3]<br>
     * Button 5 - 手首が右回転する[2]<br>
     * Button 6 - 手首が左回転する[1]<br><br>
     * Button 7 - 奥のコーンのゴールまでアームを伸ばす[2]<br>
     * Button 8 - 奥のキューブのゴールまでアームを伸ばす[5]<br>
     * Button 9 - 真ん中のコーンのゴールまでアームを伸ばす[3]<br>
     * Button 10 - 真ん中のキューブのゴールまでアームを伸ばす[6]<br>
     * Button 11 - 前のコーンのゴールまでアームを伸ばす[4]<br>
     * Button 12 - 前のキューブのゴールまでアームを伸ばす[7]<br>
     * Axis1 - joystickX -> 上下移動（右に傾けると上行き）[8]<br>
     * Axis2 - joystickY -> 前後移動（前に傾けると奥行き）[8]<br>
     * Axis3 - joystickZ -> 左右移動（時計回りだと右行き）[8]<br>
     * Axis4 - 負（< -0.8）の時、MoveArmMotorに切り替え[1]<br>その時、joystickXで根本の回転 joystickYで関節部分の回転<br><br>
     * No Button - すべて停止[100]<br>
     */
    @Override
    public void changeState() {

        DriveState.xSpeed = -driveController.getLeftY();
        DriveState.zRotation = -driveController.getRightX();
        DriveState.driveState = DriveState.DriveStates.s_midDrive;

        LimelightState.isLimelightOn = true;
        LimelightState.limelightState = States.s_tapeDetection;

        if (driveController.getAButtonPressed()) {
            if (IntakeState.intakeExtensionState == IntakeState.IntakeExtensionStates.s_closeIntake) {
                IntakeState.intakeExtensionState = IntakeState.IntakeExtensionStates.s_openIntake;
            } else {
                IntakeState.intakeExtensionState = IntakeState.IntakeExtensionStates.s_closeIntake;
            }

        }

        //RT: intake, LT: outtake
        if (driveController.getRightTriggerAxis() > 0.5) {
            IntakeState.intakeState = IntakeState.RollerStates.s_intakeGamePiece;
        } else if (driveController.getLeftTriggerAxis() > 0.5) {
            IntakeState.intakeState = IntakeState.RollerStates.s_outtakeGamePiece;
            IntakeState.intakeExtensionState = IntakeExtensionStates.s_openIntake;
        } else {
            IntakeState.intakeState = IntakeState.RollerStates.s_stopRoller;
        }

        final double joystickX = -1 * Tools.deadZoneProcess(joystick.getRawAxis(0));
        final double joystickY = 1 * Tools.deadZoneProcess(joystick.getRawAxis(1));
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

        SmartDashboard.putBoolean("test", joystick.getRawButton(5));

        if (getSeveralRawButtonPressed(new int[]{2, 6, 7, 8, 9, 10, 11, 12}) || getSeveralRawButtonReleased(new int[]{2, 6, 7, 8, 9, 10, 11, 12})) {
            ArmState.resetPidController = true;
            ArmState.targetHeight = ArmState.actualHeight;
            ArmState.targetDepth = ArmState.actualDepth;
            ArmState.relayToGoalOver = false;
            ArmState.relayToInitOver = false;
            ArmState.targetToGoalOver = false;
            ArmState.firstRelayToIntakeOver = false;
            ArmState.secondRelayToIntakeOver = false;
            isBasicRelayOver = false;
        }

        if (getSeveralRawButton(new int[]{7, 8, 9, 10, 11, 12})) {
            ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
        }

        if (joystick.getRawAxis(3) < -0.8) {
            // 各アームの角度をコントローラーで変える -> もっとも直感的
            ArmState.armState = ArmState.ArmStates.s_moveArmMotor;
            ArmState.rootSpeed = joystickX;
            ArmState.jointSpeed = joystickY;
        } else if (joystick.getRawButton(7)) {
            // 奥のコーンのゴールまでアームを伸ばす
            if (!ArmState.relayToGoalOver) {
                ArmState.targetHeight = ArmConst.RelayPointToGoalHeight;
                ArmState.targetDepth = ArmConst.RelayPointToGoalDepth;
            } else {
                ArmState.targetHeight = LimelightConst.TopGoalHeight - ArmConst.RootHeightFromGr;
                ArmState.targetDepth = ArmState.TargetDepth.TopCone;
            }
        } else if (joystick.getRawButton(9)) {
            // 真ん中のコーンのゴールまでアームを伸ばす
            if (!ArmState.relayToGoalOver) {
                ArmState.targetHeight = ArmConst.RelayPointToGoalHeight;
                ArmState.targetDepth = ArmConst.RelayPointToGoalDepth;
            } else {
                ArmState.targetHeight = LimelightConst.MiddleGoalHeight - ArmConst.RootHeightFromGr;
                if (LimelightState.tv) {
                    if (85 < LimelightState.armToGoal && LimelightState.armToGoal < 120) {
                        ArmState.targetDepth = 0.6 * LimelightState.armToGoal + 40;
                        // if (ArmState.isAtTarget() && ArmState.isAtLeftAndRightTarget()) {
                        //     HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
                        // }

                    }else if(50 < LimelightState.armToGoal && LimelightState.armToGoal <= 85) {
                        ArmState.targetDepth = 0.6 * LimelightState.armToGoal + 40 -3;
                    }


                } else {
                    ArmState.targetDepth = ArmState.TargetDepth.MiddleCone;
                }
            }
        } else if (joystick.getRawButton(11)) {
            // 前のコーンのゴールまでアームを伸ばす
                ArmState.targetHeight = LimelightConst.BottomGoalHeight - ArmConst.RootHeightFromGr;
                ArmState.targetDepth = ArmState.TargetDepth.BottomCone;
        } else if (joystick.getRawButton(8)) {
            // 奥のキューブのゴールまでアームを伸ばす
            if (!ArmState.relayToGoalOver) {
                ArmState.targetHeight = ArmConst.RelayPointToGoalHeight;
                ArmState.targetDepth = ArmConst.RelayPointToGoalDepth;
            } else {
                ArmState.targetHeight = CameraConst.TopGoalHeight - ArmConst.RootHeightFromGr;
                ArmState.targetDepth = ArmState.TargetDepth.TopCube;
            }
        } else if (joystick.getRawButton(10)) {
            // 真ん中のキューブのゴールまでアームを伸ばす
            if (!ArmState.relayToGoalOver) {
                ArmState.targetHeight = ArmConst.RelayPointToGoalHeight;
                ArmState.targetDepth = ArmConst.RelayPointToGoalDepth;
            } else {
                ArmState.targetHeight = CameraConst.MiddleGoalHeight - ArmConst.RootHeightFromGr;
                ArmState.targetDepth = ArmState.TargetDepth.MiddleCube;
            }
        } else if (joystick.getRawButton(12)) {
            // 前のキューブのゴールまでアームを伸ばす
            ArmState.targetHeight = CameraConst.BottomGoalHeight - ArmConst.RootHeightFromGr;
            ArmState.targetDepth = ArmState.TargetDepth.BottomCube;
            if (ArmState.targetToGoalOver) {
                HandState.grabHandState = HandState.GrabHandStates.s_releaseHand;
            }
        } else if (driveController.getPOV() == 90) {
            ArmState.armState = ArmState.ArmStates.s_moveArmToSpecifiedPosition;
            ArmState.targetHeight = -7;
            ArmState.targetDepth = ArmState.TargetDepth.SubStation;
        } else {
            if (joystick.getPOV() == 0) {
                adjustArmPosition(0, ArmConst.TargetModifyRatio);
            } else if (joystick.getPOV() == 180) {
                adjustArmPosition(0, -ArmConst.TargetModifyRatio);
            } else if (joystick.getPOV() == 90) {
                adjustArmPosition(ArmConst.TargetModifyRatio, 0);
            } else if (joystick.getPOV() == 270) {
                adjustArmPosition(-ArmConst.TargetModifyRatio, 0);
            } else if (joystick.getPOV() == 45) {
                adjustArmPosition(ArmConst.TargetModifyRatio, ArmConst.TargetModifyRatio);
            } else if (joystick.getPOV() == 135) {
                adjustArmPosition(ArmConst.TargetModifyRatio, -ArmConst.TargetModifyRatio);
            } else if (joystick.getPOV() == 225) {
                adjustArmPosition(-ArmConst.TargetModifyRatio, -ArmConst.TargetModifyRatio);
            } else if (joystick.getPOV() == 315) {
                adjustArmPosition(-ArmConst.TargetModifyRatio, ArmConst.TargetModifyRatio);
            }
        }

        if (joystick.getRawAxis(3) < -0.8) {
            ArmState.armState = ArmState.ArmStates.s_moveArmMotor;
            ArmState.rootSpeed = joystickX;
            ArmState.jointSpeed = joystickY;
            // 奥のコーン
        }

        isBasicRelayOver |= ArmState.actualDepth < 30;
        if (joystick.getRawButton(2)) {
            Util.Calculate.setIntakeWithRelay();
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_movetomiddle;
            HandState.rotateState = HandState.RotateStates.s_turnHandBack;
        }

        if (driveController.getBButton()) {
            ArmState.moveLeftAndRightArmState = ArmState.MoveLeftAndRightArmState.s_limelightTracking;
        }

       

    }

    /**
     * @param Height : ターゲットのX座標[cm]
     * @param Depth  : ターゲットのZ座標[cm]
     *               この関数に座標の値域を記述する
     * @return 入力の座標が正しいか[boolean]
     */
    private static boolean isNewTargetPositionInLimit(double Height, double  Depth) {
        double length = Math.sqrt(Math.pow(Height, 2) + Math.pow(Depth, 2));

        boolean isInOuterBorder = length < ArmConst.TargetPositionOuterLimit;
        boolean isOutInnerBorder = length > ArmConst.TargetPositionInnerLimit;
        // boolean isInDepthLimit = Depth > -23;

        SmartDashboard.putBoolean("InLimit", isInOuterBorder && isOutInnerBorder);

        // TODO XButtonでコントロールする時のターゲット座標の制限を考える
        return isInOuterBorder && isOutInnerBorder;
    }

    /**
     * 複数のボタンのうちどれかが押されているか判定する
     * 複数のボタンのgetButtonをORで
     *
     * @param buttonIds ボタンの番号の配列
     * @return 複数のボタンのうちどれか一つがgetButtonでtrueかどうか
     */
    private boolean getSeveralRawButton(int[] buttonIds) {
        boolean flag = false;
        for (int buttonIdx : buttonIds) {
            flag |= joystick.getRawButton(buttonIdx);
        }
        return flag;
    }

    /**
     * 複数のボタンのうちどれかが押されたか判定する
     * 複数のボタンのgetButtonPressedをORで
     *
     * @param buttonIds ボタンの番号の配列
     * @return 複数のボタンのうちどれか一つがgetButtonPressedでtrueかどうか
     */
    private boolean getSeveralRawButtonPressed(int[] buttonIds) {
        boolean flag = false;
        for (int buttonIdx : buttonIds) {
            flag |= joystick.getRawButtonPressed(buttonIdx);
        }
        return flag;
    }

    /**
     * 複数のボタンのうちどれかが離されたか判定する
     * 複数のボタンのgetButtonReleasedをORで
     *
     * @param buttonIds ボタンの番号の配列
     * @return 複数のボタンのうちどれか一つがgetButtonReleasedでtrueかどうか
     */
    private boolean getSeveralRawButtonReleased(int[] buttonIds) {
        boolean flag = false;
        for (int buttonIdx : buttonIds) {
            flag |= joystick.getRawButtonReleased(buttonIdx);
        }
        return flag;
    }

    public static void adjustArmPosition(double diffH, double diffD) {
        ArmState.armState = ArmState.ArmStates.s_adjustArmPosition;
        if (isNewTargetPositionInLimit(ArmState.targetHeight + diffH, ArmState.targetDepth + diffD)) {
            ArmState.targetHeight += diffH;
            ArmState.targetDepth += diffD;
        } else {
            ArmState.targetHeight = ArmState.actualHeight;
            ArmState.targetDepth = ArmState.actualDepth;
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