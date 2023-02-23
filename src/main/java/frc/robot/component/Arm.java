package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.Tools;


public class Arm implements Component{
    private final PIDController pidForRoot;
    private final PIDController pidForJoint;

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

        pidForRoot = new PIDController(Const.Arms.kP1, Const.Arms.kI1, Const.Arms.kD1);
        pidForJoint = new PIDController(Const.Arms.kP2, Const.Arms.kI2, Const.Arms.kD2);

        pidForRoot.setIntegratorRange( -0.04 / Const.Arms.kI1, 0.04 / Const.Arms.kI1);
        pidForRoot.setTolerance(2);
        pidForJoint.setTolerance(1);
    }

    /**
     * PIDで移動する
     * moveArmToSpecifiedPositionで実行
     * */
    private void pidControlArm() {
        topMotor.set(ControlMode.PercentOutput, pidForJoint.calculate(State.armActualJointAngle) + State.armTopMotorFeedforward);
        underMotor.set(ControlMode.PercentOutput, pidForRoot.calculate(State.armActualRootAngle) + State.armUnderMotorFeedforward);
//        underMotor.set(ControlMode.PercentOutput, pidForTheta1.calculate(State.armActualTheta1) + 0.2535 * Math.cos(State.armActualTheta1));
    }

    /**
     * アームを静止させる
     * feedforwardを計算してモーターに入力
     * */
    private void stopArm() {
        topMotor.set(State.armTopMotorFeedforward);
        underMotor.set(State.armUnderMotorFeedforward);
//        topMotor.stopMotor();
//        underMotor.set(0.2535 * Math.cos(State.armActualTheta1));
    }

    /**
     * コントローラーでアームを動かす
     * */
    private void rotationControlArm (double top, double under) {
        topMotor.set(ControlMode.PercentOutput, top);
        underMotor.set(ControlMode.PercentOutput, under);
    }

    private boolean isArmAtTarget () {
        return pidForRoot.atSetpoint() && pidForJoint.atSetpoint();
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
        // motorのencodeからアームの実際のX,Z座標を計算
        State.armActualRootAngle = getE1Angle(encoder1.get());
        State.armActualJointAngle = getE2Angle(encoder2.get());
        State.armActualHeight = Tools.calculateX(State.armActualRootAngle, State.armActualJointAngle);
        State.armActualDepth = Tools.calculateZ(State.armActualRootAngle, State.armActualJointAngle);

        // armがターゲットの座標に到着したか
        State.isArmAtTarget = isArmAtTarget();

        // フィードフォワードを計算する
        State.armTopMotorFeedforward = Tools.calculateTopMotorFeedforward(State.armActualRootAngle, State.armActualJointAngle);
        State.armUnderMotorFeedforward = Tools.calculateUnderMotorFeedforward(State.armActualRootAngle, State.armActualJointAngle);
        State.armTopMotorFeedforward = Tools.changeTorqueToMotorInput(State.armTopMotorFeedforward / Const.Arms.TopMotorGearRatio);
        State.armUnderMotorFeedforward = Tools.changeTorqueToMotorInput(State.armUnderMotorFeedforward / Const.Arms.TopUnderGearRatio);
    }

    @Override
    public void applyState() {

        pidForRoot.setSetpoint(State.armTargetRootAngle);
        pidForJoint.setSetpoint(State.armTargetJointAngle);

        if(State.resetArmPidController){
            pidForRoot.reset();
            pidForJoint.reset();
        }

        if(State.resetArmEncoder) {
            encoder1.reset();
            encoder2.reset();
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
