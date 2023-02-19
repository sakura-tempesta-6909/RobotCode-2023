package frc.robot.subClass;

import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.Map;

public class Tools {
    /**
     * @param theta1 : readSensorで取得した実際の角度[deg]
     * @param theta2 : readSensorで取得した実際の角度[deg]
     * @return X座標[cm]
     * */
    public static double calculateX(double theta1, double theta2) {
        theta1 = Math.toRadians(theta1);
        theta2 = Math.toRadians(theta2);
        double theta3 = theta1 + theta2;
        double l1 = Const.Arms.FirstArmLength;
        double l2 = Const.Arms.SecondArmLength;

        return l1 * Math.sin(theta1) - l2 * Math.sin(theta3);
    }

    /**
     * @param theta1 : readSensorで取得した実際の角度[deg]
     * @param theta2 : readSensorで取得した実際の角度[deg]
     * @return Z座標[cm]
     * */
    public static double calculateZ(double theta1, double theta2) {
        theta1 = Math.toRadians(theta1);
        theta2 = Math.toRadians(theta2);
        double theta3 = theta1 + theta2;
        double l1 = Const.Arms.FirstArmLength;
        double l2 = Const.Arms.SecondArmLength;

        return l1 * Math.cos(theta1) - l2 * Math.cos(theta3);
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
     * @param X : ターゲットのX座標[cm]
     * @param Z : ターゲットのZ座標[cm]
     * X,Zからtheta1, theta2 を計算
     * @return アームのターゲットの角度[deg]
     */
    public static Map<String, Double> calculateThetas(double X, double Z) {
        double l1 = Const.Arms.FirstArmLength;
        double l2 = Const.Arms.SecondArmLength;

        double theta2 = Math.acos((Math.pow(l1, 2) + Math.pow(l2, 2)
                - Math.pow(X, 2) - Math.pow(Z, 2)) / (2 * l1 * l2));

        double tX = l1 - l2 * Math.cos(theta2);
        double tY = l2 * Math.sin(theta2);
        double r = Math.sqrt(Math.pow(tX, 2) + Math.pow(tY, 2));

        double alphaSin = Math.asin(tX / r);
        double theta1Sin = Math.asin(Z / r) - alphaSin;
        double alphaCos = Math.acos(tX / r);
        double theta1Cos = Math.acos(Z / r) + alphaCos;

        double theta1 = Math.abs(Tools.calculateX(theta1Sin, theta2) - X) > Math.abs(Tools.calculateX(theta1Cos, theta2) - X)
                ? theta1Cos : theta1Sin;

        Map<String, Double> thetas = new HashMap<String, Double>();
        thetas.put("theta1", Math.toDegrees(theta1));
        thetas.put("theta2", Math.toDegrees(theta2));

        return thetas;
    }

    /**
     * @param theta1 : readSensorで取得した実際の角度[deg]
     * @param theta2 : readSensorで取得した実際の角度[deg]
     * underMotorのフィードフォワードを計算
     * それぞれのアームの重心をConstから取得
     * @return モーメント[N*cm]
     * */
    public static double calculateUnderMotorFeedforward (double theta1, double theta2) {
        theta1 = Math.toRadians(theta1);
        theta2 = Math.toRadians(theta2);
        double theta3 = theta1 + theta2;
        double l1 = Const.Arms.FirstArmLength;
        double l2 = Const.Arms.SecondArmLength;
        double fb = Const.Arms.FirstArmBarycenter; //FirstBarycenter -> fb
        double sb = Const.Arms.SecondArmBarycenter; //SecondBarycenter -> sb
        double m1 = Const.Arms.FirstArmMass;
        double m2 = Const.Arms.SecondArmMass;

        double ffMomentForFirstArm = fb * m1 * Math.sin(theta1);
        double ffMomentForSecondArm = l1 * m2 * Math.sin(theta1) - sb * m2 * Math.sin(theta3);

        //TODO feedforwardでmotor.setに渡す値はトルクの計算が必要
        return ffMomentForFirstArm + ffMomentForSecondArm;
    }

    /**
     * @param theta1 : readSensorで取得した実際の角度[deg]
     * @param theta2 : readSensorで取得した実際の角度[deg]
     * topMotorのフィードフォワードを計算
     * 重心などをConstから取得
     * @return モーメント[N*cm]
     * */
    public static double calculateTopMotorFeedforward (double theta1, double theta2) {
        theta1 = Math.toRadians(theta1);
        theta2 = Math.toRadians(theta2);
        double theta3 = theta1 + theta2;
        double sb = Const.Arms.SecondArmBarycenter; //SecondBarycenter -> sb
        double m2 = Const.Arms.SecondArmMass;

        return sb * m2 * Math.sin(theta3 - Math.PI);
    }

    /**
     * NEOモーターのトルクとRPMの関係を利用 <a href="https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf">NEOのデータシート</a>
     * [注意] NEOモーターに合わせて出力する
     * @param torque : トルク[N*cm] = モーメント / Const.Arms.[Under/Top]MotorGearRatio（ギア比に合わせて入力）
     * @return motor.setへの入力[-1.0, 1.0] (CANSparkMax)
     * */
    public static double changeTorqueToMotorInput (double torque) {
        return 1 - torque / Const.Arms.MotorMaxTorque;
    }
}
