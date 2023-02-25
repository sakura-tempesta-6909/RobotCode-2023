package frc.robot.subClass;

import java.util.HashMap;
import java.util.Map;

/**
 * <a href="https://github.com/sakura-tempesta-6909/RobotCode-2023/files/10830447/default.pdf">計算式などはこちらから</a>
 * // TODO calculateAnglesの誤差が大きくなる範囲があるかもなので検証する
 * // TODO 計算式の再検証 theta_dash + 90 = theta（thetaにtheta_dash + 90を代入）
 * */
public class ArmTools {
    /**
     * @param RootAngle : readSensorで取得した実際の角度[deg]
     * @param JointAngle : readSensorで取得した実際の角度[deg]
     * @return Height座標[cm]
     * */
    public static double calculateHeight(double RootAngle, double JointAngle) {
        RootAngle = Math.toRadians(RootAngle);
        JointAngle = Math.toRadians(JointAngle);
        double SumAngle = RootAngle + JointAngle;
        double l1 = Const.Arm.FirstArmLength;
        double l2 = Const.Arm.SecondArmLength;

        return l1 * Math.sin(RootAngle) - l2 * Math.sin(SumAngle);
    }

    /**
     * @param RootAngle : readSensorで取得した実際の角度[deg]
     * @param JointAngle : readSensorで取得した実際の角度[deg]
     * @return Depth座標[cm]
     * */
    public static double calculateDepth(double RootAngle, double JointAngle) {
        RootAngle = Math.toRadians(RootAngle);
        JointAngle = Math.toRadians(JointAngle);
        double SumAngle = RootAngle + JointAngle;
        double l1 = Const.Arm.FirstArmLength;
        double l2 = Const.Arm.SecondArmLength;

        return l1 * Math.cos(RootAngle) - l2 * Math.cos(SumAngle);
    }

    private static final double deadZoneThreshold = 0.05;

    /**
     * 不感帯処理関数
     * 絶対値がdeadZoneThreshold未満のものを淘汰
     * @param input : コントローラーの値を入力
     * @return 不感帯処理を施したinput
     * */
    public static double deadZoneProcess(double input) {
        if(Math.abs(input) < deadZoneThreshold) return 0.0;
        else return input;
    }

    /**
     * @param Height : ターゲットのX座標[cm]
     * @param Depth : ターゲットのZ座標[cm]
     * X,ZからRootAngle, JointAngle を計算
     * @return アームのターゲットの角度[deg]
     */
    public static Map<String, Double> calculateAngles(double Height, double Depth) {
        double l1 = Const.Arm.FirstArmLength;
        double l2 = Const.Arm.SecondArmLength;

        double JointAngle = Math.acos((Math.pow(l1, 2) + Math.pow(l2, 2)
                - Math.pow(Height, 2) - Math.pow(Depth, 2)) / (2 * l1 * l2));

        double tX = l1 - l2 * Math.cos(JointAngle);
        double tY = l2 * Math.sin(JointAngle);
        double r = Math.sqrt(Math.pow(tX, 2) + Math.pow(tY, 2));

        double alphaSin = Math.asin(tX / r);
        double RootAngleSin = Math.asin(Depth / r) - alphaSin;
        double alphaCos = Math.acos(tX / r);
        double RootAngleCos = Math.acos(Depth / r) + alphaCos;

        double RootAngle = Math.abs(ArmTools.calculateHeight(RootAngleSin, JointAngle) - Height) > Math.abs(ArmTools.calculateHeight(RootAngleCos, JointAngle) - Height)
                ? RootAngleCos : RootAngleSin;

        Map<String, Double> angles = new HashMap<String, Double>();
        angles.put("RootAngle", Math.toDegrees(RootAngle));
        angles.put("JointAngle", Math.toDegrees(JointAngle));

        return angles;
    }

    /**
     * @param RootAngle : readSensorで取得した実際の角度[deg]
     * @param JointAngle : readSensorで取得した実際の角度[deg]
     * underMotorのフィードフォワードを計算
     * それぞれのアームの重心をConstから取得
     * @return モーメント[N*cm]
     * */
    public static double calculateRootMotorFeedforward(double RootAngle, double JointAngle) {
        RootAngle = Math.toRadians(RootAngle);
        JointAngle = Math.toRadians(JointAngle);
        double SumAngle = RootAngle + JointAngle;
        double l1 = Const.Arm.FirstArmLength;
        double l2 = Const.Arm.SecondArmLength;
        double b1 = Const.Arm.FirstArmBarycenter; //FirstBarycenter -> fb
        double b2 = Const.Arm.SecondArmBarycenter; //SecondBarycenter -> sb
        double m1 = Const.Arm.FirstArmMass;
        double m2 = Const.Arm.SecondArmMass;

        double ffMomentForFirstArm = b1 * m1 * Math.sin(RootAngle);
        double ffMomentForSecondArm = l1 * m2 * Math.sin(RootAngle) - b2 * m2 * Math.sin(SumAngle);

        return ffMomentForFirstArm + ffMomentForSecondArm;
    }

    /**
     * @param RootAngle : readSensorで取得した実際の角度[deg]
     * @param JointAngle : readSensorで取得した実際の角度[deg]
     * topMotorのフィードフォワードを計算
     * 重心などをConstから取得
     * @return モーメント[N*cm]
     * */
    public static double calculateJointMotorFeedforward(double RootAngle, double JointAngle) {
        RootAngle = Math.toRadians(RootAngle);
        JointAngle = Math.toRadians(JointAngle);
        double SumAngle = RootAngle + JointAngle;
        double b2 = Const.Arm.SecondArmBarycenter;
        double m2 = Const.Arm.SecondArmMass;

        return b2 * m2 * Math.sin(SumAngle - Math.PI);
    }

    /**
     * NEOモーターのトルクとRPMの関係を利用 <a href="https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf">NEOのデータシート</a>
     * [注意] NEOモーターに合わせて出力する
     * @param torque : トルク[N*cm] = モーメント / Const.Arms.[Root/Joint]MotorGearRatio（ギア比に合わせて入力）
     * @return motor.setへの入力[-1.0, 1.0] (CANSparkMax)
     * */
    public static double changeTorqueToMotorInput (double torque) {
        return 1 - torque / Const.Arm.MotorMaxTorque;
    }
}
