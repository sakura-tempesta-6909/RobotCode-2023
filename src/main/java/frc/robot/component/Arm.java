package frc.robot.component;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.States.State;
import frc.robot.consts.ArmConst;
import frc.robot.consts.DriveConst;
import frc.robot.subClass.Tools;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subClass.Util;


public class Arm implements Component {
    private final CANSparkMax rootMotor, jointMotor;
    private final SparkMaxPIDController pidForRoot, pidForJoint;
    private final SparkMaxPIDController leftAndRightArmPidController;
    private final RelativeEncoder leftAndRightArmEncoder;
    private final CANSparkMax moveLeftAndRightMotor;

    public Arm() {
        jointMotor = new CANSparkMax(ArmConst.Ports.jointMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        rootMotor = new CANSparkMax(ArmConst.Ports.rootMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        jointMotor.setInverted(false);
        rootMotor.setInverted(true);
        rootMotor.setIdleMode(IdleMode.kCoast);
        jointMotor.setIdleMode(IdleMode.kBrake);

        pidForRoot = rootMotor.getPIDController();
        pidForJoint = jointMotor.getPIDController();

        pidForRoot.setP(ArmConst.P_R);
        pidForRoot.setI(ArmConst.I_R);
        pidForRoot.setD(ArmConst.D_R);
        pidForRoot.setIMaxAccum(ArmConst.IMax_R, 0);
        pidForRoot.setSmartMotionMaxVelocity(3000, 0);
        // pidForRoot.setSmartMotionAccelStrategy(accelStrategy, slotID)

        
        pidForRoot.setP(ArmConst.P_R_1, 1);
        pidForRoot.setI(ArmConst.I_R_1, 1);
        pidForRoot.setD(ArmConst.D_R_1, 1);
        pidForRoot.setIMaxAccum(ArmConst.IMax_R_1, 1);
        pidForRoot.setOutputRange(-.3, .3, 1);

        pidForJoint.setP(ArmConst.P_J);
        pidForJoint.setI(ArmConst.I_J);
        pidForJoint.setD(ArmConst.D_J);
        pidForJoint.setIMaxAccum(ArmConst.IMax_J, 0);
        pidForJoint.setOutputRange(-.5, .5);

        
        pidForJoint.setP(ArmConst.P_J_1, 1);
        pidForJoint.setI(ArmConst.I_J_1, 1);
        pidForJoint.setD(ArmConst.D_J_1, 1);
        pidForJoint.setIMaxAccum(ArmConst.IMax_J_1, 1);
        pidForJoint.setOutputRange(-.5, .5, 1);

        moveLeftAndRightMotor = new CANSparkMax(ArmConst.Ports.MoveLeftAndRightMotor, MotorType.kBrushless);
        leftAndRightArmPidController = moveLeftAndRightMotor.getPIDController();
        leftAndRightArmEncoder = moveLeftAndRightMotor.getEncoder();
        leftAndRightArmPidController.setP(ArmConst.P_MID);
        leftAndRightArmPidController.setI(ArmConst.I_MID);
        leftAndRightArmPidController.setD(ArmConst.D_MID);
        leftAndRightArmPidController.setIMaxAccum(ArmConst.IMax_MID, 0);
        leftAndRightArmPidController.setOutputRange(-.1, .1);
        moveLeftAndRightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 7.5f);
        moveLeftAndRightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -7.5f);
        rootMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) calculateRootRotationFromAngle(90));
        // jointMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) calculateJointRotationFromAngle(-90));


    }


    /**
     * PIDで移動する
     * feedforwardが必要かに応じてコメントアウトを外す
     * s_moveArmToSpecifiedPositionで実行
     */
    private void pidControlArm(double targetRootAngle, double targetJointAngle) {
        if(State.Arm.isAtTarget()) {
            fixPositionWithFF();
        } else {
            // feedforwardなし
            pidForRoot.setReference(calculateRootRotationFromAngle(targetRootAngle), CANSparkMax.ControlType.kPosition);
            // pidForJoint.setReference(calculateJointRotationFromAngle(targetJointAngle), CANSparkMax.ControlType.kPosition);

            // feedforwardあり
            // pidForRoot.setReference(calculateRootRotationFromAngle(targetRootAngle), CANSparkMax.ControlType.kPosition, 0, State.Arm.rootMotorFeedforward, ArbFFUnits.kPercentOut);
            pidForJoint.setReference(calculateJointRotationFromAngle(targetJointAngle), CANSparkMax.ControlType.kPosition, 0, State.Arm.jointMotorFeedforward, ArbFFUnits.kPercentOut);
        }
    }

    /**
     * PIDで移動する
     * feedforwardが必要かに応じてコメントアウトを外す
     * s_moveArmToSpecifiedPositionで実行
     */
    private void adjustArmPosition(double targetRootAngle, double targetJointAngle) {
        // feedforwardなし
        pidForRoot.setReference(calculateRootRotationFromAngle(targetRootAngle), CANSparkMax.ControlType.kPosition, 1);
        // pidForJoint.setReference(calculateJointRotationFromAngle(targetJointAngle), CANSparkMax.ControlType.kPosition, 1);

        // feedforwardあり
        // pidForRoot.setReference(calculateRootRotationFromAngle(targetRootAngle), CANSparkMax.ControlType.kPosition, 0, State.Arm.rootMotorFeedforward, ArbFFUnits.kPercentOut, 1);
        pidForJoint.setReference(calculateJointRotationFromAngle(targetJointAngle), CANSparkMax.ControlType.kPosition, 1, State.Arm.jointMotorFeedforward, ArbFFUnits.kPercentOut);
    }

    /**
     * コントローラーでアームを動かす
     * feedforwardが必要かに応じてコメントアウトを外す
     * @param joint jointモーターのスピード [-1,1]
     * @param root  rootモーターのスピード [-1,1]
     * s_moveArmMotorで実行
     */
    private void rotationControlArm(double joint, double root) {
        // feedforwardなし
        rootMotor.set(root * ArmConst.RootMotorMoveRatio);
        jointMotor.set(joint * ArmConst.JointMotorMoveRatio);

        // feedforwardあり
        // rootMotor.set(root * Const.Arm.RootMotorMoveRatio + State.Arm.rootMotorFeedforward);
        // jointMotor.set(joint * Const.Arm.JointMotorMoveRatio + State.Arm.jointMotorFeedforward);
    }

    /**
     * アームをその位置で止める
     * feedforwardが必要かに応じてコメントアウトを外す
     * s_fixArmPositionで実行
     * */
    private void fixPositionWithFF() {
        // adjustArmPosition(State.Arm.actualRootAngle, State.Arm.actualJointAngle);
        // feedforwardなし
        // rootMotor.set(0.0);
        // jointMotor.set(0.0);

        // 常に一定の数値をfeedforwardとして入れる
        rootMotor.set(ArmConst.ConstantRootMotorFF);

        // feedforwardあり
        // rootMotor.set(State.Arm.rootMotorFeedforward);
        jointMotor.set(State.Arm.jointMotorFeedforward);
    }

    /**
     * 根本NEOモーターの回転数から根本アームの角度を計算（根本の回転数 * 360）
     * @param rotation encoderから取得したPosition（モーターの回転数）
     * @return 根本アームの角度[deg]
     * */
    private double calculateRootAngleFromRotation(double rotation) {
        return rotation / ArmConst.RootMotorGearRatio * 360 + ArmConst.RootHomePosition;
    }

    /**
     * 関節部分NEOモーターの回転数から先端アームの角度を計算（根本の回転数 * 360）
     * @param rotation encoderから取得したPosition（関節部分NEOモーターの回転数）
     * @return 先端アームの角度[deg]
     * */
    private double calculateJointAngleFromRotation(double rotation) {
        return rotation / ArmConst.JointMotorGearRatio * 360 + ArmConst.JointHomePosition;
    }

    /**
     * 根本アームの角度から根本NEOモーターの回転数を計算（根本NEOモーターの必要な角度 / 360 = 必要な回転数）
     * @param angle 根本アームの角度[deg]
     * @return 根本NEOモーターの回転数
     * */
    private double calculateRootRotationFromAngle(double angle) {
        return  (angle - ArmConst.RootHomePosition) / 360 * ArmConst.RootMotorGearRatio;
    }

    /**
     * 先端アームの角度から関節部分NEOモーターの回転数を計算（関節部分NEOモーターの必要な角度 / 360 = 必要な回転数）
     * @param angle 先端アームの角度[deg]
     * @return 関節部分NEOモーターの回転数
     * */
    private double calculateJointRotationFromAngle(double angle) {
        return (angle - ArmConst.JointHomePosition) / 360 * ArmConst.JointMotorGearRatio;
    }

    private double calculateLeftAndRightAngleFromRotation(double rotation) {
        return rotation / ArmConst.LeftAndRightArmGearRatio * 360;
    }

    /**
     *度数から回転数への変換
     * @param angle 変換する角度の度数
     * @return 変換された回転数
     */
    private double calculateLeftAndRightRotationFromAngle(double angle) {
        return angle * ArmConst.LeftAndRightArmGearRatio / 360;
    }

    public void moveRightArm(double moveLeftAndRightSpeed) {
        moveLeftAndRightMotor.set(moveLeftAndRightSpeed);
    }

    public void moveLeftArm(double moveLeftAndRightSpeed) {
        moveLeftAndRightMotor.set(moveLeftAndRightSpeed);
    }

    public void stopLeftAndRightArm() {
        moveLeftAndRightMotor.set(DriveConst.Speeds.Neutral);
    }

    /**
     * アームを真ん中に動かす
     * 目標値（真ん中）を0とする
     */
    public void moveArmToMiddle() {
        leftAndRightArmPidController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    public void pidControlTargetTracking() {
        State.Arm.targetMoveLeftAndRightAngle = State.tx + State.Arm.actualLeftAndRightAngle;
        if (!State.tv) {
            moveRightArm(0.05);
        } else {
            leftAndRightArmPidController.setReference(calculateLeftAndRightRotationFromAngle(-State.Arm.targetMoveLeftAndRightAngle), CANSparkMax.ControlType.kPosition);
        }

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
        State.Arm.actualRootAngle = calculateRootAngleFromRotation(rootMotor.getEncoder().getPosition());
        State.Arm.actualJointAngle = calculateJointAngleFromRotation(jointMotor.getEncoder().getPosition());
        State.Arm.actualHeight = Tools.calculateHeight(State.Arm.actualRootAngle, State.Arm.actualJointAngle);
        State.Arm.actualDepth = Tools.calculateDepth(State.Arm.actualRootAngle, State.Arm.actualJointAngle);

        // armが左右に動いてる時の位置（角度）
        State.Arm.actualLeftAndRightAngle = calculateLeftAndRightAngleFromRotation(leftAndRightArmEncoder.getPosition());

        // feedforwardを計算する
        // TODO コーンを持っているかによってrequiredTorqueを変える
        double jointRequiredTorque = Tools.calculateJointMotorFeedforward(State.Arm.actualRootAngle, State.Arm.actualJointAngle) / ArmConst.JointMotorGearRatio;
        double rootRequiredTorque = Tools.calculateRootMotorFeedforward(State.Arm.actualRootAngle, State.Arm.actualJointAngle) / ArmConst.JointMotorGearRatio;
        State.Arm.jointMotorFeedforward = Tools.changeTorqueToMotorInput(jointRequiredTorque);
        State.Arm.rootMotorFeedforward = Tools.changeTorqueToMotorInput(rootRequiredTorque);

        State.Arm.relayToGoalOver |= Util.Calculate.isOverRelayToGoal(State.Arm.actualHeight, State.Arm.actualDepth);
        State.Arm.relayToInitOver |= Util.Calculate.isOverRelayToInit(State.Arm.actualHeight, State.Arm.actualDepth);
        
        SmartDashboard.putNumber("actual leftright angle", State.Arm.actualLeftAndRightAngle);

        SmartDashboard.putNumber("cc", rootMotor.getOutputCurrent());
        SmartDashboard.putNumber("tmp", rootMotor.getMotorTemperature());
        SmartDashboard.putNumber("out", rootMotor.getAppliedOutput());
    }

    @Override
    public void applyState() {

        if (State.Arm.resetPidController) {
            pidForRoot.setIAccum(0.0);
            pidForJoint.setIAccum(0.0);
        }

        if (State.Arm.resetEncoder) {
            jointMotor.getEncoder().setPosition(0.0);
            rootMotor.getEncoder().setPosition(0.0);
        }

        if (State.Arm.isMoveLeftAndRightEncoderReset) {
            leftAndRightArmEncoder.setPosition(7.5);
        }

        switch (State.Arm.state) {
            case s_moveArmToSpecifiedPosition:
                pidControlArm(State.Arm.targetRootAngle, State.Arm.targetJointAngle);
                break;
            case s_adjustArmPosition:
                adjustArmPosition(State.Arm.targetRootAngle, State.Arm.targetJointAngle);
                break;
            case s_moveArmMotor:
                rotationControlArm(State.Arm.jointSpeed, State.Arm.rootSpeed);
                break;
            case s_fixArmPosition:
                fixPositionWithFF();
                break;
        }

        switch (State.moveLeftAndRightArmState) {
            case s_moveRightMotor:
                moveRightArm(ArmConst.Speeds.MoveLeftAndRightMotor);
                break;
            case s_moveLeftMotor:
                moveLeftArm(-ArmConst.Speeds.MoveLeftAndRightMotor);
                break;
            case s_fixLeftAndRightMotor:
                stopLeftAndRightArm();
                break;
            case s_movetomiddle:
                moveArmToMiddle();
                break;
            case s_limelightTracking:
                pidControlTargetTracking();
                break;
        }
    }
}
