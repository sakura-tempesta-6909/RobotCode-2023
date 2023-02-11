package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.Tools;


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
        pidForTheta1.setTolerance(2);
        pidForTheta2.setTolerance(1);
    }

    private void pidControlArm() {
        topMotor.set(ControlMode.PercentOutput, pidForTheta2.calculate(State.armActualTheta2));
        underMotor.set(ControlMode.PercentOutput, pidForTheta1.calculate(State.armActualTheta1) + 0.2535 * Math.cos(State.armActualTheta1));
    }

    private void stopArm() {
        topMotor.stopMotor();
        //feedforward only
        underMotor.set(0.2535 * Math.cos(State.armActualTheta1));
    }

    private void rotationControlArm (double top, double under) {
        topMotor.set(ControlMode.PercentOutput, top);
        underMotor.set(ControlMode.PercentOutput, under);
    }

    private boolean isArmAtTarget () {
        return pidForTheta1.atSetpoint() && pidForTheta2.atSetpoint();
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
        State.armActualTheta1 = getE1Angle(encoder1.get());
        State.armActualTheta2 = getE2Angle(encoder2.get());
        State.armActualAxisX = Tools.calculateX(State.armActualTheta1, State.armActualTheta2);
        State.armActualAxisZ = Tools.calculateZ(State.armActualTheta1, State.armActualTheta2);

        State.isArmAtTarget = isArmAtTarget();
    }

    @Override
    public void applyState() {

        pidForTheta1.setSetpoint(State.armTargetTheta1);
        pidForTheta2.setSetpoint(State.armTargetTheta2);

        if(State.resetPidController){
            pidForTheta1.reset();
            pidForTheta2.reset();
        }

        switch (State.armState) {
            case s_moveArmToSpecifiedPosition:
                pidControlArm();
                break;
            case s_moveArmMotor:
                rotationControlArm(State.rightX, State.leftY);
                break;
            case s_fixArmPosition:
                stopArm();
                break;
        }
    }
}
