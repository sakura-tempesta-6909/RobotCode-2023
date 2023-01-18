package frc.robot.Component;

import frc.robot.State;
import frc.robot.SubClass.Const;

import java.util.HashMap;
import java.util.Map;

public class Arm implements Component{

    public Arm() {

    }

    private double calculateX(double theta1, double theta2) {
        double theta3 = theta1 + theta2;
        double l1 = Const.Arms.FirstArmLength;
        double l2 = Const.Arms.SecondArmLength;

        return l1 * Math.sin(theta1) - l2 * Math.sin(theta3);
    }

    private double calculateZ(double theta1, double theta2) {
        double theta3 = theta1 + theta2;
        double l1 = Const.Arms.FirstArmLength;
        double l2 = Const.Arms.SecondArmLength;

        return l1 * Math.cos(theta1) - l2 * Math.cos(theta3);
    }

    public Map<String, Double> calculateThetas(double X, double Z) {
        double l1 = Const.Arms.FirstArmLength;
        double l2 = Const.Arms.SecondArmLength;

        double theta2 = Math.acos(Math.pow(l1, 2) + Math.pow(l2, 2)
                - Math.pow(X, 2) - Math.pow(Z, 2) / 2 * l1 * l2);

        double tX = l1 - l2 * Math.cos(theta2);
        double tY = Const.Arms.SecondArmLength * Math.sin(theta2);
        double r = Math.sqrt(Math.pow(tX, 2) + Math.pow(tY, 2));

        double alphaSin = Math.asin(tX / r);
        double theta1Sin = Math.asin(Z / r) - alphaSin;
        double alphaCos = Math.acos(tX / r);
        double theta1Cos = Math.acos(Z / r) + alphaCos;

        double theta1 = Math.abs(calculateX(theta1Sin, theta2) - X) > Math.abs(calculateX(theta1Cos, theta2) - X)
                ? theta1Cos : theta1Sin;

        Map<String, Double> thetas = new HashMap<String, Double>();
        thetas.put("theta1", theta1);
        thetas.put("theta2", theta2);

        return thetas;
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void readSensors() {

    }

    @Override
    public void applyState() {
        Map<String, Double> thetas = calculateThetas(State.armTargetX, State.armTargetZ);

        //モーターをこの通りに動かす
        //TODO 何回も計算する必要はないので、そこらへん考えておく
    }
}
