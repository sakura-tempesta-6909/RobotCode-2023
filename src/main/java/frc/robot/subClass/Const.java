package frc.robot.subClass;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

/** 常に決まっている数値(定数)をまとめたファイル */
public class Const {
    public static final class Ports {
        public static final int DriveController = 0;
        public static final int OperateController = 1;
        public static final int Joystick = 2;

        public static final int DriveRightFront = 8;
        public static final int DriveLeftFront = 10;
        public static final int DriveRightBack = 7;
        public static final int DriveLeftBack = 9;

        public static final int HandSolenoid = 0;

        public static final int IntakeSolenoid = 0;

        public static final int RightRoller = 4;
        public static final int LeftRoller = 3;
        public static final int BottomRoller = 11;

        public static final int MoveLeftAndRightMotor = 5;
        public static int HandRotationMotor = 6;
    }

    public static final class Speeds {
        public static final double Neutral = 0;

        public static final double FastDrive = 0.8;
        public static final double MidDrive = 0.5;
        public static final double SlowDrive = 0.3;

        public static final double HandRotationSpeed = 0.5;
    }

    public static final class Calculation {
        public static final class Limelight {
            // limelightの情報
            /** Limelightの横の角度の最大 */
            public static final double LimelightMaxAngleWidth = 27;

            /** Limelightの角度(度数法) */
            public static final double LimelightMountAngleDegrees = 34.5;

            /** Limelightの高さ */
            public static final double LimelightLensHeight = 81.5; //  [cm]

            /** LimelightからArmまでの距離 */
            public static final double LimelightToArm = 0; // [cm]


            // ターゲットの情報
            /** ターゲットの高さ */
            public static final double GoalHeight = 166; // [cm]


            /** 奥のコーンのゴールの高さ[cm] */
            public static final double BackGoalHeight = 180;
            /** 真ん中のコーンのゴールの高さ[cm] */
            public static final double MiddleGoalHeight = 100;
            /** 前のコーンのゴールの高さ[cm] */
            public static final double FrontGoalHeight = 0;

            /** 手前から奥のターゲットまでの距離 */
            public static final double FrontGoalToBackGoal = 43; // [cm]

            public static final class PID {
                public static final double LimelightDriveP = 0.08;
                public static final double LimelightDriveI = 0.02;
                public static final double LimelightDriveD = 0;
            }

        }


        public static final class Camera {
            //　Cameraの情報
            /** 　Cameraの縦の中心 */
            public static final double CameraCenterHeight = 320;

            /** Cameraの横の中心 */
            public static final double CameraCenterWidth = 240;

            /** Cameraの横の比 */
            public static final double VerticalRatio = 4;

            /** Cameraの縦の比 */
            public static final double HorizontalRatio = 3;

            /** Cameraの視野角の半分 */
            public static final double FieldOfViewHalf = 34.25;

            /** Cameraの角度(度数法) */
            public static final double CameraMountAngleDegrees = 0;

            /** Cameraの高さ */
            public static final double CameraLensHeight = 42.5; // [cm]

            /** CameraからArmまでの距離 */
            public static final double CameraToArm = 0; // [cm]


            //ターゲットの情報
            /** ターゲットの高さ */
            public static final double GoalHeight = 76.5; // [cm]


            /** 奥のキューブのゴールの高さ[cm] */
            public static final double BackGoalHeight = 180;
            /** 真ん中のキューブのゴールの高さ[cm] */
            public static final double MiddleGoalHeight = 100;
            /** 前のキューブのゴールの高さ[cm] */
            public static final double FrontGoalHeight = 0;


            //計算
            /** 斜辺の長さ */
            public static final double ObliqueLine = Math.pow(VerticalRatio, 2) + Math.pow(HorizontalRatio, 2);

            /** 縦の角度の最大 */
            public static final double ThetaMaxHeight = FieldOfViewHalf * VerticalRatio / Math.sqrt(ObliqueLine);

            /** 横の角度の最大 */
            public static final double ThetaMaxWidth = FieldOfViewHalf * HorizontalRatio / Math.sqrt(ObliqueLine);

            /** 縦の焦点距離 */
            public static final double FocalLengthHeight = CameraCenterHeight / Math.tan(Math.toRadians(ThetaMaxHeight));
            /** 横の焦点距離 */
            public static final double FocalLengthWeight = CameraCenterWidth / Math.tan(Math.toRadians(ThetaMaxWidth));

            public static final class PID {
                public static final double CameraDriveP = 0.05;
                public static final double CameraDriveI = 0.0007;
                public static final double CameraDriveD = 0.003;
            }

        }


        public static double SideRollerOuttakeSpeed = 0.5;
        public static double SideRollerIntakeSpeed = 0.5;
        public static double BottomRollerOuttakaeSpeed = 0.5;
        public static double BottomRollerIntakeSpeed = 0.5;

        public static double HandRotationSpeed = 0.3;
    }

    public static final class Drive {

        public static final class PID {
            public static final double LengthThreshold = 50;
            public static final double LossTolerance = 0.5;
            public static final double PointsPerLength = 20;
            public static final int LongSlotIdx = 0;
            public static final int ShortSlotIdx = 1;
            public static final TalonSRXConfiguration DriveRight = new TalonSRXConfiguration();
            public static final TalonSRXConfiguration DriveLeft = new TalonSRXConfiguration();

            public static void init() {
                DriveRight.slot0.kP = 0.051;
                DriveRight.slot0.kI = 0.000006;
                DriveRight.slot0.kD = 0.00054;
                DriveRight.slot0.maxIntegralAccumulator = 1023 * 0.014 / DriveRight.slot0.kI;

                DriveLeft.slot0.kP = 0.048;
                DriveLeft.slot0.kI = 0.000009;
                DriveLeft.slot0.kD = 0.00054;
                DriveLeft.slot0.maxIntegralAccumulator = 1023 * 0.014 / DriveLeft.slot0.kI;

                DriveRight.slot1.kP = 0.2;
                DriveRight.slot1.kI = 0.004;
                DriveRight.slot1.kD = 0.000;
                DriveRight.slot1.maxIntegralAccumulator = 1023 * 0.1 / DriveRight.slot1.kI;

                DriveLeft.slot1.kP = 0.2;
                DriveLeft.slot1.kI = 0.0004;
                DriveLeft.slot1.kD = 0.000;
                DriveLeft.slot1.maxIntegralAccumulator = 1023 * 0.1 / DriveLeft.slot1.kI;
            }
        }
    }

    public static final class Arm {

        public static final class Ports {
            public static final int topMotor = 0;
            public static final int underMotor = 0;
        }

        /** 根本のアームの長さ[cm] */
        public static final double RootArmLength = 45.0;
        /** 先端のアームの長さ[cm] */
        public static final double HeadArmLength = 45.5;
        /** 根本のアームの重心の位置[cm]（根本からの距離） */
        public static final double RootArmBarycenter = 45.0;
        /** 先端のアームの重心の位置[cm]（関節部分からの距離） */
        public static final double HeadArmBarycenter = 45.0;
        /** 根本のアームの重さ[N] 注意 - [N]=[kg*9.8] */
        public static final double RootArmMass = 0.0 * 9.8;
        /** 先端のアームの重さ[N] 注意 - [N]=[kg*9.8] */
        public static final double HeadArmMass = 0.0 * 9.8;
        /** ターゲットの変更の速さ（コントローラーの値に乗算する） */
        public static final double TargetModifyRatio = 1;
        /** 掴んだ後に先端を持ちあげる高さ[cm] */
        public static final double TakeUpLengthAfterGrab = 20.0;

        // TODO slotの導入 - コーンを持っているかどうかで値を変える
        /** 根本のNEOモーターのPIDのP */
        public static final double P_R = 0.04;
        /** 根本のNEOモーターのPIDのI */
        public static final double I_R = 10e-5;
        /** 根本のNEOモーターのPIDのD */
        public static final double D_R = 0.00;
        /** 根本のNEOモーターの積分値の最大 */
        public static final double IMax_R = 10e3;

        /** 関節部分のNEOモーターのPIDのP */
        public static final double P_J = 0.03;
        /** 関節部分のNEOモーターのPIDのI */
        public static final double I_J = 5e-7;
        /** 関節部分のNEOモーターのPIDのD */
        public static final double D_J = 0.000;
        /** 関節部分のNEOモーターの積分値の最大 */
        public static final double IMax_J = 10e5;
        /** 根本のNEOモーターのフィードフォワードの値（定数） -> 固いため計算不要の際に */
        public static final double RootMotorFF = 0.03;

        /** アームを左右に動かすモーターのPIDのP */
        public static final double P_MID = 0.0;
        /** アームを左右に動かすモーターのPIDのI */
        public static final double I_MID = 0.0;
        /** アームを左右に動かすモーターのPIDのD */
        public static final double D_MID = 0.0;
        /** アームを左右に動かすモーターの積分値の最大 */
        public static final double IMax_MID = 0.0;
        /** handのモーターののPIDのP */
        public static final double P_HANDR = 0.0;
        /** handのモーターののPIDのI */
        public static final double I_HANDR = 0.0;
        /** handのモーターののPIDのD */
        public static final double D_HANDR = 0.0;
        /** handのモーターのの積分値の最大値 */
        public static final double IMax_HANDR = 0.0;

        /**
         * NEOモーターの最大トルク 注意! [N*cm] = [N*m] * 100
         * <a href="https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf">NEOのデータシートを参照</a>
         */
        public static final double MotorMaxTorque = 2.6 * 100;
        /** ターゲットの座標の閾値（外側）[cm] */
        public static final double TargetPositionOuterLimit = RootArmLength + HeadArmLength - 2;
        /** ターゲットの座標の閾値（内側）[cm] */
        public static final double TargetPositionInnerLimit = RootArmLength - HeadArmLength + 2;
        /** 関節部分のNEOモーターのギア比 */
        public static final double JointMotorGearRatio = 4.0 * 5.0 * 40.0 / 12.0;
        /** 根本のNEOモーターのギア比 */
        public static final double RootMotorGearRatio = 3.0 * 5.0 * 5.0 * 40.0 / 12.0;
        /** アームを左右に動かす時のギア比 */
        public static final double LeftAndRightArmGearRatio = 1;
        /** 関節部分のモーターをコントローラーで動かす時の最大の速さ */
        public static final double JointMotorMoveRatio = 0.09;
        /** 根本のモーターをコントローラーで動かす時の最大の速さ */
        public static final double RootMotorMoveRatio = 0.5;
        /** PIDコントロールの誤差の許容量[deg] 注意! isArmAtTargetの判定に用いているだけ */
        public static final double PIDAngleTolerance = 0.1;

        /** アームの理想的な高さ */
        public static final double InitialHeight = 0;
        /** アームの理想的な奥行き */
        public static final double InitialDepth = 0;

        /** アームの根本の高さ[cm] -> 座標の原点の高さ */
        public static final double RootHeight = 40;

    }

    public static final class Hand {
        /** アームを左右に動かす時のギア比 */
        public static final double HandGearRatio = 1;
    }


    public static final class MQTT {
        public static final String Broker = "tcp://raspberrypi.local:1883";
        public static final String Topic = "robot/data/main";
        public static final int MaxRetry = 100;
        public static final String ClientId = "robot/test";
    }

    public static final class GrabGamePiecePhase {
        /** インテイクのゲームピースを掴むアームの高さ */
        public static final double armIntakeHeight = 0;
        /** インテイクのゲームピースを掴むアームの奥行き */
        public static final double armIntakeDepth = 0;
    }

    public static void ConstInit() {

    }
}
