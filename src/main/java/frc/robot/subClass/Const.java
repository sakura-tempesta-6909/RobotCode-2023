package frc.robot.subClass;

public class Const {
    public static final class Ports {
        public static final int DriveController = 0;
        public static final int OperateController = 1;

        public static final int DriveRightFront = 0;
        public static final int DriveLeftFront = 1;
        public static final int DriveRightBack = 2;
        public static final int DriveLeftBack = 3;
    }

    public static final class Speeds {
        public static final double Neutral = 0;

        public static final double FastDrive = 0.8;
        public static final double MidDrive = 0.5;
        public static final double SlowDrive = 0.3;

    }

    public static final class Calculation {
        public static final class Camera {
            public static final double CameraCenterHight = 320;
            public static final double CameraCenterWeight = 240;
            public static final double VerticalRatio = 4;
            public static final double HorizontalRatio = 3;
            public static final double FieldOfViewHalf = 34.25;
            public static final double ObliqueLine = Math.pow(VerticalRatio, 2) + Math.pow(HorizontalRatio, 2);
            public static final double ThetaMaxHight = FieldOfViewHalf * VerticalRatio / Math.sqrt(ObliqueLine);
            public static final double ThetaMaxWeigt = FieldOfViewHalf * HorizontalRatio / Math.sqrt(ObliqueLine);

            public static final double FocalLengthHight = CameraCenterHight / Math.tan(Math.toRadians(ThetaMaxHight));
            public static final double FocalLengthWeight = CameraCenterWeight / Math.tan(Math.toRadians(ThetaMaxWeigt));

            public static final double CameraMountAngleDegrees = 0;
            public static final double CameraLensHeightCentis = 42.5;
            public static final double GoalHightCentis = 76.5;
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

        /** 関節部分のNEOモーターのPIDのP */
        public static final double P_J = 0.03;
        /** 関節部分のNEOモーターのPIDのI */
        public static final double I_J = 5e-7;
        /** 関節部分のNEOモーターのPIDのD */
        public static final double D_J = 0.000;
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
        /** 関節部分のモーターをコントローラーで動かす時の最大の速さ */
        public static final double JointMotorMoveRatio = 0.09;
        /** 根本のモーターをコントローラーで動かす時の最大の速さ */
        public static final double RootMotorMoveRatio = 0.5;
        /** PIDコントロールの誤差の許容量[deg] 注意! isArmAtTargetの判定に用いているだけ */
        public static final double PIDAngleTolerance = 0.1;
    }

    public static void ConstInit() {

    }
}
