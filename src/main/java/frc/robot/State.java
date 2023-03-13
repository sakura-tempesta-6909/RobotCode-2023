package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.State.Hand.RotateState;
import frc.robot.mode.ArmMode;
import frc.robot.mode.DriveMode;
import frc.robot.mode.Mode;
import frc.robot.mode.TestMode;
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
    public static double limelightXSpeed;
    public static boolean pidLimelightReset;

    /** カメラの横の中心座標 */
    public static double cameraCenterWidth;
    /** カメラの盾の中心座標 */
    public static double cameraCenterHeight;
    public static double cameraXSpeed;

    /** Autonomousの遷移の種類　[ A, B, C ] のいずれか*/
    public static String autonomousPhaseTransType;

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

        public static void StateInit() {
        }

        public static void StateReset() {
            grabHandState = GrabHandState.s_grabHand;
            rotateState = RotateState.s_stopHand;
        }
    }

    public static class Drive {

        public static States state;
        /** PID時のターゲット[cm] 正が前向き */
        public static double targetLength;
        /** 左右のモーターの位置[cm] 正が前向き */
        public static double rightLength, leftLength;
        /** ロボットが目標の位置にいるか */
        public static boolean isAtTarget;
        /** arcadeDrive用の引数 */
        public static double xSpeed, zRotation;

        public static boolean resetPIDController, resetPosition;

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
            targetLength = 0.0;
            xSpeed = 0.0;
            zRotation = 0.0;
            rightLength = 0.0;
            leftLength = 0.0;
        }

        public static void StatesReset() {
            state = States.s_stopDrive;
            resetPosition = false;
            resetPIDController = false;
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
        /** アームがターゲットについているか（ターゲットとの誤差がConst.Arm.PIDAngleTolerance以下か） */
        public static boolean isAtTarget;

        public static double moveLeftAndRightMotor;
        /** アームを左右に動かす時の位置 */
        public static double leftAndRightPositionAngle;

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
        }


        public enum States {
            /** アームを指定した場所に移動させる */
            s_moveArmToSpecifiedPosition,
            /** アームの支点を動かす */
            s_moveArmMotor,
            /** アームをその場で固定する */
            s_fixArmPosition,
        }

        public static void StatesInit() {
            //init armMode value
            Arm.targetHeight = 0.0;
            Arm.targetDepth = 0.0;

            Arm.actualHeight = 0.0;
            Arm.actualDepth = 0.0;

            Arm.targetRootAngle = 0.0;
            Arm.targetJointAngle = 0.0;

            Arm.actualRootAngle = 0.0;
            Arm.actualJointAngle = 0.0;

            Arm.jointSpeed = 0.0;
            Arm.rootSpeed = 0.0;

            Arm.rootMotorFeedforward = 0.0;
            Arm.jointMotorFeedforward = 0.0;

            Arm.moveLeftAndRightMotor = 0.0;

            Arm.isAtTarget = false;
        }

        public static void StatesReset() {
            Arm.state = Arm.States.s_fixArmPosition;
            Arm.resetPidController = false;
            Arm.resetEncoder = false;

            // TODO どれくらい引くかを計測する
            TargetDepth.TopCorn = 101.0 - 40.0;
            TargetDepth.MiddleCorn = 58.0 - 20.0;
            TargetDepth.BottomCorn = 30.0;

            TargetDepth.TopCube = 101.0 - 40.0;
            TargetDepth.MiddleCube = 58.0 - 20.0;
            TargetDepth.BottomCube = 30.0;
        }
    }

    public static Map<String, Double> voltage = new HashMap<>();
    public static RotateState rotateState;


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

        voltage = new HashMap<>();

        StateReset();
    }

    /**
     * コントローラーから手を離している間の状態
     */
    public static void StateReset() {
        intakeState = RollerState.s_stopRoller;
        rotateState = RotateState.s_stopHand;
        moveLeftAndRightArmState = MoveLeftAndRightArmState.s_fixLeftAndRightMotor;
        pidLimelightReset = false;

        autonomousPhaseTransType = Util.getConsole("AutonomousPhaseTransition");

        // reset arm states
        Drive.StatesReset();
        Arm.StatesReset();
        Hand.StateReset();
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
    }


    public enum Modes {
        k_drive(new DriveMode()),
        k_arm(new ArmMode()),
        k_test(new TestMode());

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
