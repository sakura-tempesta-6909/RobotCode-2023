package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.ArmTools;


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

        pidForRoot = new PIDController(Const.Arm.kP1, Const.Arm.kI1, Const.Arm.kD1);
        pidForJoint = new PIDController(Const.Arm.kP2, Const.Arm.kI2, Const.Arm.kD2);

        pidForRoot.setIntegratorRange( -0.04 / Const.Arm.kI1, 0.04 / Const.Arm.kI1);
        pidForRoot.setTolerance(2);
        pidForJoint.setTolerance(1);
    }

    /**
     * PIDで移動する
     * moveArmToSpecifiedPositionで実行
     * */
    private void pidControlArm() {
        topMotor.set(ControlMode.PercentOutput, pidForJoint.calculate(State.Arm.actualJointAngle) + State.Arm.topMotorFeedforward);
        underMotor.set(ControlMode.PercentOutput, pidForRoot.calculate(State.Arm.actualRootAngle) + State.Arm.underMotorFeedforward);
    }

    /**
     * アームを静止させる
     * feedforwardを計算してモーターに入力
     * */
    private void stopArm() {
        topMotor.set(State.Arm.topMotorFeedforward);
        underMotor.set(State.Arm.underMotorFeedforward);
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
        return (x) / Const.Arm.Encoder1CountPerRotation;
    }

    public double getE2Angle(double x){
        return (x) / Const.Arm.Encoder2CountPerRotation;
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
        State.Arm.actualRootAngle = getE1Angle(encoder1.get());
        State.Arm.actualJointAngle = getE2Angle(encoder2.get());
        State.Arm.actualHeight = ArmTools.calculateHeight(State.Arm.actualRootAngle, State.Arm.actualJointAngle);
        State.Arm.actualDepth = ArmTools.calculateDepth(State.Arm.actualRootAngle, State.Arm.actualJointAngle);

        // armがターゲットの座標に到着したか
        State.Arm.isArmAtTarget = isArmAtTarget();

        // フィードフォワードを計算する
        State.Arm.topMotorFeedforward = ArmTools.calculateJointMotorFeedforward(State.Arm.actualRootAngle, State.Arm.actualJointAngle);
        State.Arm.underMotorFeedforward = ArmTools.calculateRootMotorFeedforward(State.Arm.actualRootAngle, State.Arm.actualJointAngle);
        State.Arm.topMotorFeedforward = ArmTools.changeTorqueToMotorInput(State.Arm.topMotorFeedforward / Const.Arm.TopMotorGearRatio);
        State.Arm.underMotorFeedforward = ArmTools.changeTorqueToMotorInput(State.Arm.underMotorFeedforward / Const.Arm.TopUnderGearRatio);
    }

    @Override
    public void applyState() {

        pidForRoot.setSetpoint(State.Arm.targetRootAngle);
        pidForJoint.setSetpoint(State.Arm.targetJointAngle);

        if(State.Arm.resetArmPidController) {
            pidForRoot.reset();
            pidForJoint.reset();
        }

        if(State.Arm.resetArmEncoder) {
            encoder1.reset();
            encoder2.reset();
        }

        switch (State.Arm.state) {
            case s_moveArmToSpecifiedPosition:
                pidControlArm();
                break;
            case s_moveArmMotor:
                rotationControlArm(State.Arm.rightX, State.Arm.leftY);
                break;
            case s_fixArmPosition:
                stopArm();
                break;
        }
    }
}
