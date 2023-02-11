package frc.robot.subClass;

import java.util.HashMap;
import java.util.Map;

public class Tools {
    /**
     * theta1, theta2 -> 度数法
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
     * theta1, theta2 -> 度数法
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
     * */
    public static double deadZoneProcess(double input) {
        if(Math.abs(input) < deadZoneThreshold) return 0.0;
        else return input;
    }

    /**
     * X,Zからtheta1, theta2 (度数法) を計算
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
}
