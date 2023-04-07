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
     * 先端アームの長さ[cm]（45.5はハンドの付け根までの長さ）
     */
    public static final double HeadArmLength = 45.5;
    /**
     * ハンドの折れた角度（折れ具合）[deg]
     * */
    public static final double HandFoldAngle = 55.0;
    /**
     * ハンドの長さ[cm]（ハンドの付け根から先端まで）
     * */
    public static final double HandLength = 46.0;
    /**
     * ハンドが折れていることによって生じる仮想アームの折れた角度（折れ具合）[deg]
     * （正の向きに折れ曲がっている）
     * */
    public static final double VirtualArmFoldAngle = Math.toDegrees(
            Math.atan((Math.sin(Math.toRadians(HandFoldAngle)) * HandLength) /
                    (HeadArmLength + Math.cos(Math.toRadians(HandFoldAngle)) * HandLength))
    );
    /**
     * 仮想の先端アームの長さ[cm]（関節部分からハンドの先端まで）
     * */
    public static final double VirtualHeadArmLength = Math.sqrt(
            Math.pow(HeadArmLength + Math.cos(Math.toRadians(HandFoldAngle)) * HandLength, 2)
                    + Math.pow(Math.sin(Math.toRadians(HandFoldAngle)) * HandLength, 2)
    );
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
    public static final double P_R = 0.04 + 0.02;
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
    public static final double P_J = 0.03 + 0.03 + 0.03;
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

    public static final double P_R_2 = 0.00016;//31
    public static final double I_R_2 = 0;
    public static final double D_R_2 = 0;
    public static final double P_J_2 = 0.00008;
    public static final double I_J_2 = 0.0000015;
    public static final double D_J_2 = 0;

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
    public static final double P_MID = 0.010;
    /**
     * アームを左右に動かすモーターのPIDのI
     */
    public static final double I_MID = 0.000003;
    /**
     * アームを左右に動かすモーターのPIDのD
     */
    public static final double D_MID = 0.0;
    /**
     * アームを左右に動かすモーターの積分値の最大
     */
    public static final double IMax_MID = 0.0;

    public static final double P_MID_1 = 0.0003;
    public static final double I_MID_1 = 0.0;
    public static final double D_MID_1 = 0.0;
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
    public static final double IMax_HANDR = 0.03/ 0.00001;


    /**
     * NEOモーターの最大トルク 注意! [N*cm] = [N*m] * 100
     * <a href="https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf">NEOのデータシートを参照</a>
     */
    public static final double MotorMaxTorque = 2.6 * 100;
    /**
     * ターゲットの座標の閾値（外側）[cm]
     */
    public static final double TargetPositionOuterLimit = RootArmLength + VirtualHeadArmLength;
    /**
     * ターゲットの座標の閾値（内側）[cm]
     */
    public static final double TargetPositionInnerLimit = RootArmLength - VirtualHeadArmLength;
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
    public static final double JointMotorMoveRatio = 0.2;
    /**
     * 根本NEOモーターをコントローラーで動かす時の最大の速さ
     */
    public static final double RootMotorMoveRatio = 0.3;
    /**
     * PIDコントロールの誤差の許容量[cn] 注意! isArmAtTargetの判定に用いているだけ
     */
    public static final double PIDAngleTolerance = 3;

    /**
     * アームの理想的な高さ
     */
    public static final double InitialHeight = -90;
    /**
     * アームの理想的な奥行き
     */
    public static final double InitialDepth = 5;

    /**
     * アームの根本の高さ[cm]（地面から） -> 座標の原点の高さ
     */
    public static final double RootHeightFromGr = 127;

    public static final double RootHomePosition = -85.20;
    public static final double JointHomePosition = -79.32;

    public static final double RelayPointToGoalHeight = 0;
    public static final double RelayPointToGoalDepth = 60;
    public static final double RelayPointToInitHeight = -80;
    public static final double RelayPointToInitDepth = 10;
    public static final double RelayPointTolerance = 10;

    public static void ArmConstInit() {

    }
}
