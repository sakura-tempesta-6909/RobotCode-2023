package frc.robot.subClass;

import java.util.HashMap;
import java.util.Map;

public class Tools {
    /**
     * @param RootAngle : readSensorで取得した実際の角度[deg]
     * @param JointAngle : readSensorで取得した実際の角度[deg]
     * @return X座標[cm]
     * */
    public static double calculateX(double RootAngle, double JointAngle) {
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
     * @return Z座標[cm]
     * */
    public static double calculateZ(double RootAngle, double JointAngle) {
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

        double RootAngle = Math.abs(Tools.calculateX(RootAngleSin, JointAngle) - Height) > Math.abs(Tools.calculateX(RootAngleCos, JointAngle) - Height)
                ? RootAngleCos : RootAngleSin;

        Map<String, Double> thetas = new HashMap<String, Double>();
        thetas.put("RootAngle", Math.toDegrees(RootAngle));
        thetas.put("JointAngle", Math.toDegrees(JointAngle));

        return thetas;
    }

    /**
     * @param RootAngle : readSensorで取得した実際の角度[deg]
     * @param JointAngle : readSensorで取得した実際の角度[deg]
     * underMotorのフィードフォワードを計算
     * それぞれのアームの重心をConstから取得
     * @return モーメント[N*cm]
     * */
    public static double calculateUnderMotorFeedforward (double RootAngle, double JointAngle) {
        RootAngle = Math.toRadians(RootAngle);
        JointAngle = Math.toRadians(JointAngle);
        double SumAngle = RootAngle + JointAngle;
        double l1 = Const.Arm.FirstArmLength;
        double l2 = Const.Arm.SecondArmLength;
        double fb = Const.Arm.FirstArmBarycenter; //FirstBarycenter -> fb
        double sb = Const.Arm.SecondArmBarycenter; //SecondBarycenter -> sb
        double m1 = Const.Arm.FirstArmMass;
        double m2 = Const.Arm.SecondArmMass;

        double ffMomentForFirstArm = fb * m1 * Math.sin(RootAngle);
        double ffMomentForSecondArm = l1 * m2 * Math.sin(RootAngle) - sb * m2 * Math.sin(SumAngle);

        //TODO feedforwardでmotor.setに渡す値はトルクの計算が必要
        return ffMomentForFirstArm + ffMomentForSecondArm;
    }

    /**
     * @param RootAngle : readSensorで取得した実際の角度[deg]
     * @param JointAngle : readSensorで取得した実際の角度[deg]
     * topMotorのフィードフォワードを計算
     * 重心などをConstから取得
     * @return モーメント[N*cm]
     * */
    public static double calculateTopMotorFeedforward (double RootAngle, double JointAngle) {
        RootAngle = Math.toRadians(RootAngle);
        JointAngle = Math.toRadians(JointAngle);
        double SumAngle = RootAngle + JointAngle;
        double sb = Const.Arm.SecondArmBarycenter; //SecondBarycenter -> sb
        double m2 = Const.Arm.SecondArmMass;

        return sb * m2 * Math.sin(SumAngle - Math.PI);
    }

    /**
     * NEOモーターのトルクとRPMの関係を利用 <a href="https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf">NEOのデータシート</a>
     * [注意] NEOモーターに合わせて出力する
     * @param torque : トルク[N*cm] = モーメント / Const.Arms.[Under/Top]MotorGearRatio（ギア比に合わせて入力）
     * @return motor.setへの入力[-1.0, 1.0] (CANSparkMax)
     * */
    public static double changeTorqueToMotorInput (double torque) {
        return 1 - torque / Const.Arm.MotorMaxTorque;
    }
}
