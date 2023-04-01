package frc.robot.States;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.mode.*;
import frc.robot.subClass.Const;
import frc.robot.subClass.Util;

import java.util.HashMap;
import java.util.Map;

public class State {
    public static Modes mode;
    public static RollerState intakeState;
    public static IntakeExtensionState intakeExtensionState;
    public static MoveLeftAndRightArmState moveLeftAndRightArmState;

    /** ターゲットを向く時のスピード */
    public static double limelightTrackingZRotation;
    /** aprilTagを向くときのスピード */
    public static double cameraTrackingZRotation;
    /** 手前のターゲットまでの距離 */
    public static double limelightToFrontGoal; // [cm]
    /** cameraからtagまでの距離 */
    public static double cameraToTag; // [cm]
    /** cameraからみたaprilTagの縦の角度(度数法) */
    public static double aprilTagAngleHeight;
    /** cameraからみたaprilTagの横の角度(度数法) */
    public static double aprilTagAngleWidth;
    /** armからtagまでの距離 */
    public static double armToTag; // [cm]
    /** armからターゲットまでの距離 */
    public static double armToGoal; // [cm]
    /** 奥のターゲットまでの距離 */
    public static double limelightToBackGoal; // [cm]
    public static double tx;
    public static boolean tv;
    public static double limelightXSpeed;
    public static boolean pidLimelightReset;

    /** カメラの横の中心座標 */
    public static double cameraCenterWidth;
    /** カメラの盾の中心座標 */
    public static double cameraCenterHeight;
    public static double cameraXSpeed;
    public static boolean isCompressorEnable;

    /** Autonomousの遷移の種類　[ A, B, C ] のいずれか */
    public static String autonomousPhaseTransType = "C";


    public static class Hand {
        public static GrabHandState grabHandState;
        public static RotateState rotateState;

        public enum RotateState {
            /** 手首を回転させる */
            s_rightRotateHand,
            /** 手首を逆回転させる */
            s_leftRotateHand,
            /** 手首の回転を止める */
            s_stopHand,
            /** 手首を元の位置に戻す */
            s_turnHandBack,
            /** 手首を所定の位置に動かす */
            s_moveHandToSpecifiedAngle,
        }

        /** 手首の回転した度数 */
        public static double actualHandAngle = 0.0;

        public static double targetAngle = 0.0;
        public static boolean isResetHandPID = false;

        public static void StateInit() {
        }

        public static void StateReset() {
            grabHandState = GrabHandState.s_grabHand;
            rotateState = RotateState.s_stopHand;
            isResetHandPID = false;
        }
    }

    public static class Drive {

        public static States state;
        /** PID時のターゲット[cm] 正が前向き */
        public static double targetMeter;
        /** 左右のモーターの位置[cm] 正が前向き */
        public static double rightMeter, leftMeter;
        /** arcadeDrive用の引数 */
        public static double xSpeed, zRotation;

        public static boolean resetPIDController, resetPosition;

        public static boolean isMotorBrake;

        public static boolean isAtTarget() {
            boolean isLeftMotorAtTarget = Math.abs(State.Drive.leftMeter - State.Drive.targetMeter) < Const.Drive.PID.LossTolerance;
            boolean isRightMotorAtTarget = Math.abs(State.Drive.rightMeter - State.Drive.targetMeter) < Const.Drive.PID.LossTolerance;
            return isRightMotorAtTarget && isLeftMotorAtTarget;
        }

        public enum States {
            /** ロボットの速度を速くする */
            s_fastDrive,
            /** ロボットの速度を中くらいにする */
            s_midDrive,
            /** ロボットの速度を遅くする */
            s_slowDrive,
            /** ロボットの速度を0にする */
            s_stopDrive,
            /** コーンのtargetに照準を合わせる */
            s_limelightTracking,
            /** キューブのtargetに照準を合わせる */
            s_aprilTagTracking,
            /** pidで動く */
            s_pidDrive,
        }

        public static void StatesInit() {
            targetMeter = 0.0;
            xSpeed = 0.0;
            zRotation = 0.0;
            rightMeter = 0.0;
            leftMeter = 0.0;
        }

        public static void StatesReset() {
            state = States.s_stopDrive;
            resetPosition = false;
            resetPIDController = false;
            isCompressorEnable = true;
            isMotorBrake = false;
            Arm.StatesReset();
            Hand.StateReset();
        }
    }

    public static class Arm {
        /** アームのモード */
        public static States state;
        /** 計測した角度(actualRootAngle, actualJointAngle)から計算した長さの値[cm] */
        public static double actualHeight, actualDepth;
        /** ターゲットの高さ[cm] */
        public static double targetHeight;
        /** ターゲットの奥行き[cm] */
        public static double targetDepth;
        /**
         * 計測した角度の値[deg]
         * root - 根本の角度
         * joint - 関節部分の角度
         */
        public static double actualRootAngle, actualJointAngle;
        /**
         * ターゲットの座標から計算された角度の目標値[deg]
         * root - 根本の角度
         * joint - 関節部分の角度
         */
        public static double targetRootAngle, targetJointAngle;
        /** モーターの速度 */
        public static double jointSpeed;
        public static double rootSpeed;
        /** 根本のNEOモーターに必要になるfeedforwardのspeed[-1, 1] */
        public static double rootMotorFeedforward;
        /** 関節部分のNEOモーターに必要になるfeedforwardのspeed[-1, 1] */
        public static double jointMotorFeedforward;

        public static double targetMoveLeftAndRightAngle;
        public static boolean isMoveLeftAndRightEncoderReset;
        /**
         * アームがターゲット位置にいるかを判定
         * targetAngleとactualAngleの差がPIDAngleTolerance未満でtrue
         * @return jointMotorとrootMotorの両方がatSetpointかどうか
         * */
        public static boolean isAtTarget() {
            boolean isDepthAtSetpoint = Math.abs(State.Arm.targetDepth - State.Arm.actualDepth) < Const.Arm.PIDAngleTolerance;
            boolean isHeightMotorAtSetpoint = Math.abs(State.Arm.targetHeight - State.Arm.actualHeight) < Const.Arm.PIDAngleTolerance;
            return isHeightMotorAtSetpoint && isDepthAtSetpoint;
        }

        public static double moveLeftAndRightMotor;
        /** アームを左右に動かす時の位置 */
        public static double actualLeftAndRightAngle;

        /** PIDコントローラーをリセットする（Integralの値をリセットする） */
        public static boolean resetPidController;
        /** エンコーダーをリセット（その時点の位置を0と定める） */
        public static boolean resetEncoder;

        public static class TargetDepth {
            public static double TopCorn;
            public static double MiddleCorn;
            public static double BottomCorn;

            public static double TopCube;
            public static double MiddleCube;
            public static double BottomCube;

            public static double SubStation;
        }


        public enum States {
            /** アームを指定した場所に移動させる */
            s_moveArmToSpecifiedPosition,
            /** アームを微調整する */
            s_adjustArmPosition,
            /** アームの支点を動かす */
            s_moveArmMotor,
            /** アームをその場で固定する */
            s_fixArmPosition,
        }

        public static void StatesInit() {
            //init armMode value
            targetHeight = 10000.0;
            targetDepth = 10000.0;

            actualHeight = 0.0;
            actualDepth = 0.0;

            targetRootAngle = 0.0;
            targetJointAngle = 0.0;

            actualRootAngle = 0.0;
            actualJointAngle = 0.0;

            jointSpeed = 0.0;
            rootSpeed = 0.0;

            rootMotorFeedforward = 0.0;
            jointMotorFeedforward = 0.0;

            moveLeftAndRightMotor = 0.0;
        }


        public static void StatesReset() {
            state = Arm.States.s_fixArmPosition;
            resetPidController = false;
            resetEncoder = false;
            isMoveLeftAndRightEncoderReset = false;

            // TODO どれくらい引くかを計測する
            TargetDepth.TopCorn = 101.0 ;
            TargetDepth.MiddleCorn = 58.0 + 20;
            TargetDepth.BottomCorn = 30.0 + 10;

            TargetDepth.TopCube = 101.0 -10;
            TargetDepth.MiddleCube = 58.0 + 20;
            TargetDepth.BottomCube = 30.0 + 10;

            TargetDepth.SubStation = 36 + 20;
        }
    }

    public static Map<String, Double> voltage = new HashMap<>();

    /**
     * Enableされたときの状態
     */
    public static void StateInit() {
        XboxController driveController = new XboxController(Const.Ports.DriveController);
        XboxController operateController = new XboxController(Const.Ports.OperateController);
        Joystick joystick = new Joystick(Const.Ports.Joystick);

        State.mode = State.Modes.k_drive;

        Mode.addController(driveController, operateController, joystick);

        intakeExtensionState = IntakeExtensionState.s_openIntake;

        // initialize arm states
        Drive.StatesInit();
        Arm.StatesInit();
        Hand.StateInit();
        LimelightState.StateInit();

        voltage = new HashMap<>();

        StateReset();
    }

    /**
     * コントローラーから手を離している間の状態
     */
    public static void StateReset() {
        intakeState = RollerState.s_stopRoller;
        moveLeftAndRightArmState = MoveLeftAndRightArmState.s_fixLeftAndRightMotor;
        pidLimelightReset = false;
        intakeExtensionState = IntakeExtensionState.s_openIntake;

        isCompressorEnable = true;

        // reset arm states
        Drive.StatesReset();
        Arm.StatesReset();
        Hand.StateReset();
        LimelightState.StateReset();
    }

    public enum RollerState {
        /** Rollerを外向きに動かし、ゲームピースを出す */
        s_outtakeGamePiece,
        /** Rollerを内向きに動かし、ゲームピースを取り込む */
        s_intakeGamePiece,
        /** Rollerの動きを止める */
        s_stopRoller,

    }

    public enum IntakeExtensionState {
        /** Intakeを出す */
        s_openIntake,
        /** Intakeをしまう */
        s_closeIntake,
    }

    public enum GrabHandState {
        /** 物体をつかむ */
        s_grabHand,
        /** 物体を離す */
        s_releaseHand,
    }

    public enum MoveLeftAndRightArmState {
        /** アームを右に動かす */
        s_moveRightMotor,
        /** アームを左に動かす */
        s_moveLeftMotor,
        /** アームを固定する */
        s_fixLeftAndRightMotor,
        /** アームを真ん中に動かす */
        s_movetomiddle,
        s_limelightTracking,

    }


    public enum Modes {
        k_drive(new DriveMode()),
        k_arm(new ArmMode()),
        k_test(new TestMode()),
        k_config(new ConfigMode()),
        k_chargeStation(new ChargeStationMode());

        private final Mode mode;

        Modes(Mode mode) {
            this.mode = mode;
        }

        public void changeMode() {
            this.mode.changeMode();
        }

        public void changeState() {
            this.mode.changeState();
        }
    }
}
