package frc.robot.component;

import java.util.Map;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.states.ArmState;
import frc.robot.states.LimelightState;
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

        pidForRoot.setP(ArmConst.P_R_2, 2);
        pidForRoot.setI(ArmConst.I_R_2, 2);
        pidForRoot.setD(ArmConst.D_R_2, 2);

        pidForJoint.setP(ArmConst.P_J_2, 2);
        pidForJoint.setI(ArmConst.I_J_2, 2);
        pidForJoint.setD(ArmConst.D_J_2, 2);

        moveLeftAndRightMotor = new CANSparkMax(ArmConst.Ports.MoveLeftAndRightMotor, MotorType.kBrushless);
        leftAndRightArmPidController = moveLeftAndRightMotor.getPIDController();
        leftAndRightArmEncoder = moveLeftAndRightMotor.getEncoder();
        leftAndRightArmPidController.setP(ArmConst.P_MID);
        leftAndRightArmPidController.setI(ArmConst.I_MID);
        leftAndRightArmPidController.setD(ArmConst.D_MID);
        leftAndRightArmPidController.setIMaxAccum(ArmConst.IMax_MID, 0);
        leftAndRightArmPidController.setOutputRange(-.1, .1);

        leftAndRightArmPidController.setP(ArmConst.P_MID_1, 1);
        leftAndRightArmPidController.setI(ArmConst.I_MID_1, 1);
        leftAndRightArmPidController.setD(ArmConst.D_MID_1, 1);

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
            // feedforwardなし
            pidForRoot.setReference(calculateRootRotationFromAngle(targetRootAngle), CANSparkMax.ControlType.kPosition);
            // pidForJoint.setReference(calculateJointRotationFromAngle(targetJointAngle), CANSparkMax.ControlType.kPosition);

            // feedforwardあり
            // pidForRoot.setReference(calculateRootRotationFromAngle(targetRootAngle), CANSparkMax.ControlType.kPosition, 0, ArmState.rootMotorFeedforward, ArbFFUnits.kPercentOut);
            pidForJoint.setReference(calculateJointRotationFromAngle(targetJointAngle), CANSparkMax.ControlType.kPosition, 0, ArmState.jointMotorFeedforward, ArbFFUnits.kPercentOut);
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
        // pidForRoot.setReference(calculateRootRotationFromAngle(targetRootAngle), CANSparkMax.ControlType.kPosition, 0, ArmState.rootMotorFeedforward, ArbFFUnits.kPercentOut, 1);
        pidForJoint.setReference(calculateJointRotationFromAngle(targetJointAngle), CANSparkMax.ControlType.kPosition, 1, ArmState.jointMotorFeedforward, ArbFFUnits.kPercentOut);
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
        // rootMotor.set(root * ArmConst.RootMotorMoveRatio + ArmState.rootMotorFeedforward);
        // jointMotor.set(joint * ArmConst.JointMotorMoveRatio + ArmState.jointMotorFeedforward);
    }

    /**
     * アームをその位置で止める
     * feedforwardが必要かに応じてコメントアウトを外す
     * s_fixArmPositionで実行
     * */
    private void fixPositionWithFF() {
        // adjustArmPosition(ArmState.actualRootAngle, ArmState.actualJointAngle);
        // feedforwardなし
        // rootMotor.set(0.0);
        // jointMotor.set(0.0);

        // 常に一定の数値をfeedforwardとして入れる
//        rootMotor.set(ArmConst.ConstantRootMotorFF);

        // feedforwardあり
        // rootMotor.set(ArmState.rootMotorFeedforward);
//        jointMotor.set(ArmState.jointMotorFeedforward);
        pidForRoot.setReference(0, CANSparkMax.ControlType.kVelocity ,2);
        pidForJoint.setReference(0, CANSparkMax.ControlType.kVelocity ,2);

    }

    private void fixLeftAndRightArmPositionWithPID() {
        leftAndRightArmPidController.setReference(0, CANSparkMax.ControlType.kVelocity, 1);
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
        ArmState.targetMoveLeftAndRightAngle = -LimelightState.tx + ArmState.actualLeftAndRightAngle;
        if (!LimelightState.tv) {
            moveRightArm(0.05);
        } else {
            leftAndRightArmPidController.setReference(calculateLeftAndRightRotationFromAngle(ArmState.targetMoveLeftAndRightAngle), CANSparkMax.ControlType.kPosition);
            SmartDashboard.putNumber("leftRightTargetAngle", ArmState.targetMoveLeftAndRightAngle);
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
        ArmState.actualRootAngle = calculateRootAngleFromRotation(rootMotor.getEncoder().getPosition());
        ArmState.actualJointAngle = calculateJointAngleFromRotation(jointMotor.getEncoder().getPosition());
        ArmState.actualHeight = Tools.calculateHeight(ArmState.actualRootAngle, ArmState.actualJointAngle);
        ArmState.actualDepth = Tools.calculateDepth(ArmState.actualRootAngle, ArmState.actualJointAngle);

        // armが左右に動いてる時の位置（角度）
        ArmState.actualLeftAndRightAngle = calculateLeftAndRightAngleFromRotation(leftAndRightArmEncoder.getPosition());

        // feedforwardを計算する
        // TODO コーンを持っているかによってrequiredTorqueを変える
        double jointRequiredTorque = Tools.calculateJointMotorFeedforward(ArmState.actualRootAngle, ArmState.actualJointAngle) / ArmConst.JointMotorGearRatio;
        double rootRequiredTorque = Tools.calculateRootMotorFeedforward(ArmState.actualRootAngle, ArmState.actualJointAngle) / ArmConst.JointMotorGearRatio;
        ArmState.jointMotorFeedforward = Tools.changeTorqueToMotorInput(jointRequiredTorque);
        ArmState.rootMotorFeedforward = Tools.changeTorqueToMotorInput(rootRequiredTorque);

        ArmState.relayToGoalOver |= Util.Calculate.isOverRelayToGoal(ArmState.actualHeight, ArmState.actualDepth);
        ArmState.relayToInitOver |= Util.Calculate.isOverRelayToInit(ArmState.actualHeight, ArmState.actualDepth);
        ArmState.targetToGoalOver |= Util.Calculate.isOverTargetToGoal();


        SmartDashboard.putNumber("actual leftright angle", ArmState.actualLeftAndRightAngle);

        SmartDashboard.putNumber("cc", rootMotor.getOutputCurrent());
        SmartDashboard.putNumber("tmp", rootMotor.getMotorTemperature());
        SmartDashboard.putNumber("out", rootMotor.getAppliedOutput());
    }

    @Override
    public void applyState() {
         // ターゲット座標からターゲットの角度を計算する
         Map<String, Double> targetAngles = Tools.calculateAngles(ArmState.targetDepth, ArmState.targetHeight);
         Double target = targetAngles.get("RootAngle");
         if (target != null) {
             ArmState.targetRootAngle = target;
         } else {
             ArmState.targetRootAngle = ArmState.actualRootAngle;
         }
         target = targetAngles.get("JointAngle");
         if (target != null) {
             ArmState.targetJointAngle = target;
         } else {
             ArmState.targetJointAngle = ArmState.actualJointAngle;
         }

        if (ArmState.resetPidController) {
            pidForRoot.setIAccum(0.0);
            pidForJoint.setIAccum(0.0);
        }

        if (ArmState.resetEncoder) {
            jointMotor.getEncoder().setPosition(0.0);
            rootMotor.getEncoder().setPosition(0.0);
        }

        if (ArmState.isMoveLeftAndRightEncoderReset) {
            leftAndRightArmEncoder.setPosition(7.5);
        }

        switch (ArmState.armState) {
            case s_moveArmToSpecifiedPosition:
                pidControlArm(ArmState.targetRootAngle, ArmState.targetJointAngle);
                break;
            case s_adjustArmPosition:
                adjustArmPosition(ArmState.targetRootAngle, ArmState.targetJointAngle);
                break;
            case s_moveArmMotor:
                rotationControlArm(ArmState.jointSpeed, ArmState.rootSpeed);
                break;
            case s_fixArmPosition:
                fixPositionWithFF();
                break;
        }

        switch (ArmState.moveLeftAndRightArmState) {
            case s_moveRightMotor:
                moveRightArm(ArmConst.Speeds.MoveLeftAndRightMotor);
                break;
            case s_moveLeftMotor:
                moveLeftArm(-ArmConst.Speeds.MoveLeftAndRightMotor);
                break;
            case s_stopLeftAndRightMotor:
                stopLeftAndRightArm();
                break;
            case s_movetomiddle:
                moveArmToMiddle();
                break;
            case s_limelightTracking:
                pidControlTargetTracking();
                break;
            case s_fixLeftAndRightArm:
                fixLeftAndRightArmPositionWithPID();
                break;
        }
    }
}
