package frc.robot.component;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.Tools;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm implements Component {
    private final CANSparkMax rootMotor, jointMotor;
    private final SparkMaxPIDController pidForRoot, pidForJoint;

    private final CANSparkMax moveLeftAndRightMotor;

    public Arm() {
        jointMotor = new CANSparkMax(Const.Arm.Ports.topMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        rootMotor = new CANSparkMax(Const.Arm.Ports.underMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        jointMotor.setInverted(false);
        rootMotor.setInverted(true);

        pidForRoot = rootMotor.getPIDController();
        pidForJoint = jointMotor.getPIDController();

        pidForRoot.setP(Const.Arm.P_R);
        pidForRoot.setI(Const.Arm.I_R);
        pidForRoot.setD(Const.Arm.D_R);
        pidForRoot.setIMaxAccum(Const.Arm.IMax_R, 0);

        pidForJoint.setP(Const.Arm.P_J);
        pidForJoint.setI(Const.Arm.I_J);
        pidForJoint.setD(Const.Arm.D_J);
        pidForJoint.setIMaxAccum(Const.Arm.IMax_J, 0);

        moveLeftAndRightMotor = new CANSparkMax(Const.Ports.moveLeftAndRightMotor,MotorType.kBrushless);
    }

    /**
     * PIDで移動する
     * moveArmToSpecifiedPositionで実行
     */
    private void pidControlArm() {
        pidForJoint.setReference(calculateJointRotationFromAngle(State.Arm.targetJointAngle), CANSparkMax.ControlType.kPosition);
        pidForRoot.setReference(calculateRootRotationFromAngle(State.Arm.targetRootAngle), CANSparkMax.ControlType.kPosition);
    }

    /**
     * コントローラーでアームを動かす
     * @param joint jointモーターのスピード [-1,1]
     * @param root rootモーターのスピード [-1,1]
     */
    private void rotationControlArm(double joint, double root) {
        jointMotor.set(joint * Const.Arm.JointMotorMoveRatio);
        rootMotor.set(root * Const.Arm.RootMotorMoveRatio);
    }

    private boolean isArmAtTarget() {
        boolean isRootMotorAtSetpoint = Math.abs(State.Arm.targetRootAngle - State.Arm.actualRootAngle) < Const.Arm.PIDAngleTolerance;
        boolean isJointMotorAtSetpoint = Math.abs(State.Arm.targetJointAngle - State.Arm.actualJointAngle) < Const.Arm.PIDAngleTolerance;
        return isJointMotorAtSetpoint && isRootMotorAtSetpoint;
    }

    private double calculateRootAngleFromRotation(double rotation) {
        return rotation / Const.Arm.RootMotorGearRatio * 360;
    }

    private double calculateJointAngleFromRotation(double rotation) {
        return rotation / Const.Arm.JointMotorGearRatio * 360;
    }

    private double calculateRootRotationFromAngle(double angle) {
        return angle * Const.Arm.RootMotorGearRatio / 360;
    }

    private double calculateJointRotationFromAngle(double angle) {
        return angle * Const.Arm.JointMotorGearRatio / 360;
    }

    private void fixPositionWithFF() {
        rootMotor.set(0);
        jointMotor.set(0);
    }

    public void moveLeftAndRightArm(double moveLeftAndRightSpeed){
        moveLeftAndRightMotor.set(moveLeftAndRightSpeed);
    }

    public void stopLeftAndRightArm(double moveLeftAndRightSpeed){
        moveLeftAndRightMotor.set(Const.Speeds.Neutral);
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

        // armがターゲットの座標に到着したか
        State.Arm.isArmAtTarget = isArmAtTarget();
    }

    @Override
    public void applyState() {
        // フィードフォワードを計算する
        // TODO コーンを持っているかによってrequiredTorqueを変える
        double jointRequiredTorque = Tools.calculateTopMotorFeedforward(State.Arm.actualRootAngle, State.Arm.actualJointAngle) / Const.Arm.JointMotorGearRatio;
        double rootRequiredTorque = Tools.calculateUnderMotorFeedforward(State.Arm.actualRootAngle, State.Arm.actualJointAngle) / Const.Arm.JointMotorGearRatio;
        State.Arm.jointMotorFeedforward = Tools.changeTorqueToMotorInput(jointRequiredTorque);
        State.Arm.rootMotorFeedforward = Tools.changeTorqueToMotorInput(rootRequiredTorque);

        if (State.Arm.resetPidController) {
            pidForRoot.setIAccum(0);
            pidForJoint.setIAccum(0);
        }

        if (State.Arm.resetEncoder) {
            jointMotor.getEncoder().setPosition(0);
            rootMotor.getEncoder().setPosition(0);
        }

        switch (State.Arm.state) {
            case s_moveArmToSpecifiedPosition:
                pidControlArm();
                break;
            case s_moveArmMotor:
                rotationControlArm(State.Arm.rightX, State.Arm.leftY);
                break;
            case s_fixArmPosition:
                fixPositionWithFF();
                break;
        }

        switch(State.moveLeftAndRightArmState){
            case s_moveLeftAndRightMotor:
                moveLeftAndRightArm(State.Arm.moveLeftAndRightMotor);
                break;
            case s_fixLeftAndRightMotor:
                stopLeftAndRightArm(State.Arm.moveLeftAndRightMotor);
                break;
        }
    }
}
