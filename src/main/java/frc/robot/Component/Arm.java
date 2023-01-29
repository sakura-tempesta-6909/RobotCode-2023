package frc.robot.Component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.State;
import frc.robot.SubClass.Const;

import java.util.HashMap;
import java.util.Map;

public class Arm implements Component{
    private final PIDController pidForTheta1;
    private final PIDController pidForTheta2;

    private final WPI_TalonSRX underMotor;
    private final WPI_TalonSRX topMotor;

    private final Encoder encoder1;
    private final Encoder encoder2;

    public Arm() {
        encoder1 = new Encoder(2, 3);
        encoder2 = new Encoder(0,1);

        underMotor = new WPI_TalonSRX(6);
        topMotor = new WPI_TalonSRX(7);

        underMotor.setInverted(true);
        topMotor.setInverted(true);

        pidForTheta1 = new PIDController(Const.Arms.kP1, Const.Arms.kI1, Const.Arms.kD1);
        pidForTheta2 = new PIDController(Const.Arms.kP2, Const.Arms.kI2, Const.Arms.kD2);

        pidForTheta1.setIntegratorRange( -0.04 / Const.Arms.kI1, 0.04 / Const.Arms.kI1);
        pidForTheta1.setTolerance(1);
        pidForTheta2.setTolerance(1);
    }

    private double calculateX(double theta1, double theta2) {
        theta1 = Math.toRadians(theta1);
        theta2 = Math.toRadians(theta2);
        double theta3 = theta1 + theta2;
        double l1 = Const.Arms.FirstArmLength;
        double l2 = Const.Arms.SecondArmLength;

        return l1 * Math.sin(theta1) - l2 * Math.sin(theta3);
    }

    private double calculateZ(double theta1, double theta2) {
        theta1 = Math.toRadians(theta1);
        theta2 = Math.toRadians(theta2);
        double theta3 = theta1 + theta2;
        double l1 = Const.Arms.FirstArmLength;
        double l2 = Const.Arms.SecondArmLength;

        return l1 * Math.cos(theta1) - l2 * Math.cos(theta3);
    }

    private Map<String, Double> calculateThetas(double X, double Z) {
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

        double theta1 = Math.abs(calculateX(theta1Sin, theta2) - X) > Math.abs(calculateX(theta1Cos, theta2) - X)
                ? theta1Cos : theta1Sin;

        Map<String, Double> thetas = new HashMap<String, Double>();
        thetas.put("theta1", Math.toDegrees(theta1));
        thetas.put("theta2", Math.toDegrees(theta2));

        return thetas;
    }

    private void pidControlArm(double theta1, double theta2) {
        topMotor.set(ControlMode.PercentOutput, pidForTheta2.calculate(State.armTheta2, theta2));
        underMotor.set(ControlMode.PercentOutput, pidForTheta1.calculate(State.armTheta1, theta1) + 0.2535 * Math.cos(State.armTheta1));
    }

    private void rotationControlArm () {

    }

    public double getE1Angle(double x){
        return (x) / Const.Arms.Encoder1CountPerRotation;
    }

    public double getE2Angle(double x){
        return (x) / Const.Arms.Encoder2CountPerRotation;
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
        State.armTheta1 = getE1Angle(encoder1.get());
        State.armTheta2 = getE2Angle(encoder2.get());
        State.armAxisX = calculateX(State.armTheta1, State.armTheta2);
        State.armAxisZ = calculateZ(State.armTheta1, State.armTheta2);
    }

    @Override
    public void applyState() {
        Map<String, Double> thetas = calculateThetas(State.armTargetX, State.armTargetZ);

        //TODO 何回も計算する必要はないので、そこらへん考えておく
        //モーターをこの通りに動かす

        switch (State.armState) {
            case s_autoCtrl:
            case s_axisCtrl:
                pidControlArm(thetas.get("theta1"), thetas.get("theta2"));
                break;
            case s_rotationCtrl:
                rotationControlArm();
                break;
        }
    }
}
