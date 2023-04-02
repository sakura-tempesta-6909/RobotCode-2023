package frc.robot.consts;

public class ArmConst {
    public static final class Ports {
        public static final int jointMotor = 2;
        public static final int rootMotor = 1;
        public static final int MoveLeftAndRightMotor = 5;
    }

    public static final class Speeds {
        public static double MoveLeftAndRightMotor = 0.05;
    }


    /**
     * 根本アームの長さ[cm]
     */
    public static final double RootArmLength = 45.0;
    /**
     * 先端アームの長さ[cm]
     */
    public static final double HeadArmLength = 69.744; // 45.5はハンドの付け根までの長さ
    /**
     * 根本アームの重心の位置[cm]（根本からの距離）
     */
    public static final double RootArmBarycenter = 12.52;
    /**
     * 先端アームの重心の位置[cm]（関節部分からの距離）
     */
    public static final double HeadArmBarycenter = 45.43;
    /**
     * 根本アームの重さ[N] 注意 - [N]=[kg*9.8]
     */
    public static final double RootArmMass = 3.302 * 9.8;
    /**
     * 先端アームの重さ[N] 注意 - [N]=[kg*9.8]
     */
    public static final double HeadArmMass = 5.961 * 9.8;
    /**
     * ターゲットの変更の速さ（コントローラーの値に乗算する）
     */
    public static final double TargetModifyRatio = 0.5;
    /**
     * 掴んだ後に先端を持ちあげる高さ[cm]
     */
    public static final double TakeUpLengthAfterGrab = 20.0;


    // TODO slotの導入 - コーンを持っているかどうかで値を変える
    /**
     * 根本NEOモーターのPIDのP
     */
    public static final double P_R = 0.04;
    /**
     * 根本NEOモーターのPIDのI
     */
    public static final double I_R = 10e-5 / 100;
    /**
     * 根本NEOモーターのPIDのD
     */
    public static final double D_R = 0.00;
    /**
     * 根本NEOモーターの積分値の最大
     */
    public static final double IMax_R = 10e5;

    /**
     * 関節部分NEOモーターのPIDのP
     */
    public static final double P_J = 0.03 + 0.03;
    /**
     * 関節部分NEOモーターのPIDのI
     */
    public static final double I_J = 5e-7 * 30;
    /**
     * 関節部分NEOモーターのPIDのD
     */
    public static final double D_J = 10;
    /**
     * 関節部分NEOモーターの積分値の最大
     */
    public static final double IMax_J = 10e5 * 8000 / 3;

    /**
     * 根本NEOモーターのPIDのP
     */
    public static final double P_R_1 = 0.04;
    /**
     * 根本NEOモーターのPIDのI
     */
    public static final double I_R_1 = 10e-5 / 10;
    /**
     * 根本NEOモーターのPIDのD
     */
    public static final double D_R_1 = 0.00;
    /**
     * 根本NEOモーターの積分値の最大
     */
    public static final double IMax_R_1 = 10e3;

    /**
     * 関節部分NEOモーターのPIDのP
     */
    public static final double P_J_1 = 0.03 + 0.03;
    /**
     * 関節部分NEOモーターのPIDのI
     */
    public static final double I_J_1 = 5e-7 * 3;
    /**
     * 関節部分NEOモーターのPIDのD
     */
    public static final double D_J_1 = 0;
    /**
     * 関節部分NEOモーターの積分値の最大
     */
    public static final double IMax_J_1 = 10e5 * 20 / 3;

    /**
     * 根本NEOモーターのフィードフォワードの値（定数） -> 固いため計算不要の際に
     */
    public static final double ConstantRootMotorFF = 0.03;
    /**
     * 根本NEOモーターにおける先端アームのモーメントの影響力（Weight）
     */
    public static final double HeadArmFFWeightForRM = 1.0 / 4.0;
    /**
     * 根本NEOモーターにおける根本アームのモーメントの影響力（Weight）
     */
    public static final double RootArmFFWeightForRM = 1.0;
    /**
     * 根本NEOモーターのfeedforwardの強さ（Weight）
     */
    public static final double RootMotorFFWeight = 1.0 / 3.0;
    /**
     * 先端NEOモーターのfeedforwardの強さ（Weight）
     */
    public static final double JointMotorFFWeight = 1.0 / 4;


    /**
     * アームを左右に動かすモーターのPIDのP
     */
    public static final double P_MID = 0.008;
    /**
     * アームを左右に動かすモーターのPIDのI
     */
    public static final double I_MID = 0.000001;
    /**
     * アームを左右に動かすモーターのPIDのD
     */
    public static final double D_MID = 0.0;
    /**
     * アームを左右に動かすモーターの積分値の最大
     */
    public static final double IMax_MID = 0.0;
    /**
     * handのモーターののPIDのP
     */
    public static final double P_HANDR = 0.03;
    /**
     * handのモーターののPIDのI
     */
    public static final double I_HANDR = 0.00001;
    /**
     * handのモーターののPIDのD
     */
    public static final double D_HANDR = 0.0;
    /**
     * handのモーターのの積分値の最大値
     */
    public static final double IMax_HANDR = 0.0;


    /**
     * NEOモーターの最大トルク 注意! [N*cm] = [N*m] * 100
     * <a href="https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf">NEOのデータシートを参照</a>
     */
    public static final double MotorMaxTorque = 2.6 * 100;
    /**
     * ターゲットの座標の閾値（外側）[cm]
     */
    public static final double TargetPositionOuterLimit = RootArmLength + HeadArmLength;
    /**
     * ターゲットの座標の閾値（内側）[cm]
     */
    public static final double TargetPositionInnerLimit = RootArmLength - HeadArmLength;
    /**
     * 関節部分NEOモーターのギア比
     */
    public static final double JointMotorGearRatio = 4.0 * 5.0 * 40.0 / 12.0;
    /**
     * 根本NEOモーターのギア比
     */
    public static final double RootMotorGearRatio = 5.0 * 5.0 * 5.0 * 40.0 / 12.0;


    /**
     * アームを左右に動かす時のギア比
     */
    public static final double LeftAndRightArmGearRatio = 186;
    /**
     * 関節部分NEOモーターをコントローラーで動かす時の最大の速さ
     */
    public static final double JointMotorMoveRatio = 0.09;
    /**
     * 根本NEOモーターをコントローラーで動かす時の最大の速さ
     */
    public static final double RootMotorMoveRatio = 1;
    /**
     * PIDコントロールの誤差の許容量[cn] 注意! isArmAtTargetの判定に用いているだけ
     */
    public static final double PIDAngleTolerance = 5;

    /**
     * アームの理想的な高さ
     */
    public static final double InitialHeight = -60;
    /**
     * アームの理想的な奥行き
     */
    public static final double InitialDepth = 27;

    /**
     * アームの根本の高さ[cm]（地面から） -> 座標の原点の高さ
     */
    public static final double RootHeightFromGr = 127;

    public static final double RootHomePosition = -87.5;
    public static final double JointHomePosition = -52.9;
    public static final double RelayPointHeight = -20;
    public static final double RelayPointDepth = 30;

}
