package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.mode.DriveMode;
import frc.robot.mode.Mode;
import frc.robot.mode.TestMode;
import frc.robot.subClass.Const;

import java.util.HashMap;
import java.util.Map;

public class State {
    public static Modes mode;
    public static double driveXSpeed, driveZRotation;
    public static DriveState driveState;
    public static RollerState intakeState;
    public static IntakeExtensionState intakeExtensionState;
    public static GrabHandState grabHandState;


    /** ターゲットを向く時のスピード */
    public static double limelightTrackingZRotation;
    /** apriltagを向くときのスピード */
    public static double cameraTrackingZRotation;
    /** 手前のターゲットまでの距離 */
    public static double limelightToFrontGoal; // [cm]
    /** cameraからtagまでの距離 */
    public static double cameraToTag; // [cm]
    /** cameraからみたapriltagの縦の角度(度数法) */
    public static double aprilTagAngleHeight;
    /** cameraからみたapriltagの横の角度(度数法) */
    public static double aprilTagAngleWidth;
    /** armからtagまでの距離 */
    public static double armToTag; // [cm]
    /** armからターゲットまでの距離 */
    public static double armToGoal; // [cm]
    /** 奥のターゲットまでの距離 */
    public static double limelightToBackGoal; // [cm]

    public static MoveLeftAndRightArmState moveLeftAndRightArmState;

    public static class Hand {
        public static RotateState rotateState;
        public enum RotateState {
            /** 手首を回転させる */
            s_rotateHand,
            /** 手首の回転を止める */
            s_stopHand,
            /** 手首を戻す*/
            bringBackHand,
        }
        public static double handRotationAngle = 0.0;
        public static void StateInit() {
        }
        public static void StateReset() {
            rotateState = RotateState.s_stopHand;
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
        /** TODO ここに作るべき変数じゃないのでControllerのStateを作るべき */
        public static double leftY;
        public static double rightX;
        /** 根本のNEOモーターに必要になるfeedforwardのspeed[-1, 1] */
        public static double rootMotorFeedforward;
        /** 関節部分のNEOモーターに必要になるfeedforwardのspeed[-1, 1] */
        public static double jointMotorFeedforward;
        /** アームがターゲットについているか（ターゲットとの誤差がConst.Arm.PIDAngleTolerance以下か） */
        public static boolean isArmAtTarget;

        public static double moveLeftAndRightMotor;
        
        /** PIDコントローラーをリセットする（Integralの値をリセットする） */
        public static boolean resetPidController;
        /** エンコーダーをリセット（その時点の位置を0と定める） */
        public static boolean resetEncoder;
        /** limelightの値を代入 TODO 一時的な変数（実際はlimelightのStateから値を取得） */
        public static double limelightTargetHeight, limelightTargetDepth;
        /** アームを左右に動かす時の位置*/
        public static double leftAndRightPositionAngle;


        public enum States {
            /** アームを指定した場所に移動させる */
            s_moveArmToSpecifiedPosition,
            /** アームの支点を動かす */
            s_moveArmMotor,
            /** アームをその場で固定する */
            s_fixArmPosition,
        }

        public static void ArmStateInit() {
            //init armMode value
            Arm.targetHeight = 0.0;
            Arm.targetDepth = 0.0;

            Arm.actualHeight = 0.0;
            Arm.actualDepth = 0.0;

            Arm.targetRootAngle = 0.0;
            Arm.targetJointAngle = 0.0;

            Arm.actualRootAngle = 0.0;
            Arm.actualJointAngle = 0.0;

            Arm.leftY = 0.0;
            Arm.rightX = 0.0;

            Arm.limelightTargetHeight = 10.0;
            Arm.limelightTargetDepth = 80.0;

            Arm.rootMotorFeedforward = 0.0;
            Arm.jointMotorFeedforward = 0.0;
            
            Arm.moveLeftAndRightMotor = 0.0;
        }

        public static void ArmStateReset() {
            Arm.state = Arm.States.s_fixArmPosition;
            Arm.isArmAtTarget = false;
            Arm.resetPidController = false;
            Arm.resetEncoder = false;
        }
    }

    public static Map<String, Double> voltage = new HashMap<>();


    /**
     * Enableされたときの状態
     */
    public static void StateInit() {
        XboxController driveController = new XboxController(Const.Ports.DriveController);
        XboxController operateController = new XboxController(Const.Ports.OperateController);
        Mode.addController(driveController, operateController);
        grabHandState = GrabHandState.s_releaseHand;
        intakeExtensionState = IntakeExtensionState.s_openIntake;
        // initialize arm states
        Arm.ArmStateInit();
        
        voltage = new HashMap<>();
        StateReset();
    }

    /**
     * コントローラーから手を離している間の状態
     */
    public static void StateReset() {
        driveState = DriveState.s_stopDrive;
        intakeState = RollerState.s_stopRoller;
        // reset arm states
        Arm.ArmStateReset();
        Hand.StateReset();
    }

    public enum DriveState {
        /**
         * ロボットの速度を速くする
         */
        s_fastDrive,
        /**
         * ロボットの速度を中くらいにする
         */
        s_midDrive,
        /**
         * ロボットの速度を遅くする
         */
        s_slowDrive,
        /**
         * ロボットの速度を0にする
         */
        s_stopDrive,
        // targetに照準を合わせる
        s_targetTracking,
        s_apriltagTracking,



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

    public enum MoveLeftAndRightArmState{
        /** アームを左右に動かす */
        s_moveLeftAndRightMotor,
        /** アームを固定する */
        s_fixLeftAndRightMotor,
        /** アームを真ん中に動かす */
        s_movetomiddle,
    }


    public enum Modes {
        k_drive(new DriveMode()),
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
