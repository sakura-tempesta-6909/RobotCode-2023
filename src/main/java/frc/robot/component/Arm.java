package frc.robot.component;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.State;
import frc.robot.subClass.Const;
import frc.robot.subClass.Tools;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;


public class Arm implements Component {
    private final CANSparkMax rootMotor, jointMotor;
    private final SparkMaxPIDController pidForRoot, pidForJoint;
    private final SparkMaxPIDController leftAndRightArmPidController;
    private final RelativeEncoder leftAndRightArmEncoder;
    private final CANSparkMax moveLeftAndRightMotor;

    public Arm() {
        jointMotor = new CANSparkMax(Const.Arm.Ports.jointMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        rootMotor = new CANSparkMax(Const.Arm.Ports.rootMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
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

        moveLeftAndRightMotor = new CANSparkMax(Const.Ports.MoveLeftAndRightMotor, MotorType.kBrushless);
        leftAndRightArmPidController = moveLeftAndRightMotor.getPIDController();
        leftAndRightArmEncoder = moveLeftAndRightMotor.getEncoder();
        leftAndRightArmPidController.setP(Const.Arm.P_MID);
        leftAndRightArmPidController.setI(Const.Arm.I_MID);
        leftAndRightArmPidController.setD(Const.Arm.D_MID);
        leftAndRightArmPidController.setIMaxAccum(Const.Arm.IMax_MID, 0);

    }


    /**
     * PIDで移動する
     * feedforwardが必要かに応じてコメントアウトを外す
     * s_moveArmToSpecifiedPositionで実行
     */
    private void pidControlArm() {
        // feedforwardなし
        pidForRoot.setReference(calculateRootRotationFromAngle(State.Arm.targetRootAngle), CANSparkMax.ControlType.kPosition);
        // pidForJoint.setReference(calculateJointRotationFromAngle(State.Arm.targetJointAngle), CANSparkMax.ControlType.kPosition);

        // feedforwardあり
        // pidForRoot.setReference(calculateRootRotationFromAngle(State.Arm.targetRootAngle), CANSparkMax.ControlType.kPosition, 0, State.Arm.rootMotorFeedforward, ArbFFUnits.kPercentOut);
        pidForJoint.setReference(calculateJointRotationFromAngle(State.Arm.targetJointAngle), CANSparkMax.ControlType.kPosition, 0, State.Arm.jointMotorFeedforward, ArbFFUnits.kPercentOut);
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
        // rootMotor.set(root * Const.Arm.RootMotorMoveRatio);
        // jointMotor.set(joint * Const.Arm.JointMotorMoveRatio);

        // feedforwardあり
        rootMotor.set(root * Const.Arm.RootMotorMoveRatio + State.Arm.rootMotorFeedforward);
        jointMotor.set(joint * Const.Arm.JointMotorMoveRatio + State.Arm.jointMotorFeedforward);
    }

    /**
     * アームをその位置で止める
     * feedforwardが必要かに応じてコメントアウトを外す
     * s_fixArmPositionで実行
     * */
    private void fixPositionWithFF() {
        // feedforwardなし
        // rootMotor.set(0.0);
        // jointMotor.set(0.0);

        // 常に一定の数値をfeedforwardとして入れる
        rootMotor.set(Const.Arm.ConstantRootMotorFF);

        // feedforwardあり
        // rootMotor.set(State.Arm.rootMotorFeedforward);
        jointMotor.set(State.Arm.jointMotorFeedforward);
    }

    /**
     * アームがターゲット位置にいるかを判定
     * targetAngleとactualAngleの差がPIDAngleTolerance未満でtrue
     * @return jointMotorとrootMotorの両方がatSetpointかどうか
     * */
    private boolean isAtTarget() {
        boolean isRootMotorAtSetpoint = Math.abs(State.Arm.targetRootAngle - State.Arm.actualRootAngle) < Const.Arm.PIDAngleTolerance;
        boolean isJointMotorAtSetpoint = Math.abs(State.Arm.targetJointAngle - State.Arm.actualJointAngle) < Const.Arm.PIDAngleTolerance;
        return isJointMotorAtSetpoint && isRootMotorAtSetpoint;
    }

    /**
     * 根本NEOモーターの回転数から根本アームの角度を計算（根本の回転数 * 360）
     * @param rotation encoderから取得したPosition（モーターの回転数）
     * @return 根本アームの角度[deg]
     * */
    private double calculateRootAngleFromRotation(double rotation) {
        return rotation / Const.Arm.RootMotorGearRatio * 360;
    }

    /**
     * 関節部分NEOモーターの回転数から先端アームの角度を計算（根本の回転数 * 360）
     * @param rotation encoderから取得したPosition（関節部分NEOモーターの回転数）
     * @return 先端アームの角度[deg]
     * */
    private double calculateJointAngleFromRotation(double rotation) {
        return rotation / Const.Arm.JointMotorGearRatio * 360;
    }

    /**
     * 根本アームの角度から根本NEOモーターの回転数を計算（根本NEOモーターの必要な角度 / 360 = 必要な回転数）
     * @param angle 根本アームの角度[deg]
     * @return 根本NEOモーターの回転数
     * */
    private double calculateRootRotationFromAngle(double angle) {
        return angle * Const.Arm.RootMotorGearRatio / 360;
    }

    /**
     * 先端アームの角度から関節部分NEOモーターの回転数を計算（関節部分NEOモーターの必要な角度 / 360 = 必要な回転数）
     * @param angle 先端アームの角度[deg]
     * @return 関節部分NEOモーターの回転数
     * */
    private double calculateJointRotationFromAngle(double angle) {
        return angle * Const.Arm.JointMotorGearRatio / 360;
    }

    private double calculateLeftAndRightAngleFromRotation(double rotation) {
        return rotation / Const.Arm.LeftAndRightArmGearRatio * 360;
    }


    public void moveRightArm(double moveLeftAndRightSpeed) {
        moveLeftAndRightMotor.set(moveLeftAndRightSpeed);
    }

    public void moveLeftArm(double moveLeftAndRightSpeed) {
        moveLeftAndRightMotor.set(-moveLeftAndRightSpeed);
    }

    public void stopLeftAndRightArm() {
        moveLeftAndRightMotor.set(Const.Speeds.Neutral);
    }

    /**
     * アームを真ん中に動かす
     * 目標値（真ん中）を0とする
     */
    public void moveArmToMiddle() {
        leftAndRightArmPidController.setReference(0, CANSparkMax.ControlType.kPosition);
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
        State.Arm.leftAndRightPositionAngle = calculateLeftAndRightAngleFromRotation(leftAndRightArmEncoder.getPosition());

        // armがターゲットの座標に到着したか
        State.Arm.isAtTarget = isAtTarget();

        // feedforwardを計算する
        // TODO コーンを持っているかによってrequiredTorqueを変える
        double jointRequiredTorque = Tools.calculateJointMotorFeedforward(State.Arm.actualRootAngle, State.Arm.actualJointAngle) / Const.Arm.JointMotorGearRatio;
        double rootRequiredTorque = Tools.calculateRootMotorFeedforward(State.Arm.actualRootAngle, State.Arm.actualJointAngle) / Const.Arm.JointMotorGearRatio;
        State.Arm.jointMotorFeedforward = Tools.changeTorqueToMotorInput(jointRequiredTorque);
        State.Arm.rootMotorFeedforward = Tools.changeTorqueToMotorInput(rootRequiredTorque);
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

        switch (State.Arm.state) {
            case s_moveArmToSpecifiedPosition:
                pidControlArm();
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
                moveRightArm(State.Arm.moveLeftAndRightMotor);
                break;
            case s_moveLeftMotor:
                moveLeftArm(-State.Arm.moveLeftAndRightMotor);
                break;
            case s_fixLeftAndRightMotor:
                stopLeftAndRightArm();
                break;
            case s_movetomiddle:
                moveArmToMiddle();
                break;
        }
    }
}
