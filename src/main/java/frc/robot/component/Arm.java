package frc.robot.component;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.Tools;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm implements Component{
    private final PIDController pidForRoot;
    private final PIDController pidForJoint;

    private final WPI_TalonSRX underMotor;
    private final WPI_TalonSRX topMotor;

    private final Encoder encoder1;
    private final Encoder encoder2;

    private final CANSparkMax moveLeftAndRightMotor;

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

        moveLeftAndRightMotor = new CANSparkMax(Const.Ports.moveLeftAndRightMotor,MotorType.kBrushless);
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

    public void moveLeftAndRightArm(double moveLeftAndRight){
        moveLeftAndRightMotor.set(moveLeftAndRight);
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
        State.Arm.actualHeight = Tools.calculateHeight(State.Arm.actualRootAngle, State.Arm.actualJointAngle);
        State.Arm.actualDepth = Tools.calculateDepth(State.Arm.actualRootAngle, State.Arm.actualJointAngle);

        // armがターゲットの座標に到着したか
        State.Arm.isArmAtTarget = isArmAtTarget();

        // フィードフォワードを計算する
        State.Arm.topMotorFeedforward = Tools.calculateTopMotorFeedforward(State.Arm.actualRootAngle, State.Arm.actualJointAngle);
        State.Arm.underMotorFeedforward = Tools.calculateUnderMotorFeedforward(State.Arm.actualRootAngle, State.Arm.actualJointAngle);
        State.Arm.topMotorFeedforward = Tools.changeTorqueToMotorInput(State.Arm.topMotorFeedforward / Const.Arm.TopMotorGearRatio);
        State.Arm.underMotorFeedforward = Tools.changeTorqueToMotorInput(State.Arm.underMotorFeedforward / Const.Arm.TopUnderGearRatio);
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
            case s_moveLeftAndRightMotor:
                moveLeftAndRightArm(State.Arm.moveLeftAndRightMotor);
                break;
            case s_fixArmPosition:
                stopArm();
                break;
        }
    }
}
