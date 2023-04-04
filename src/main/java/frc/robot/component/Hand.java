package frc.robot.component;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.HandState;
import frc.robot.states.State;
import frc.robot.consts.ArmConst;
import frc.robot.consts.DriveConst;
import frc.robot.consts.HandConst;


public class Hand implements Component{
    private final Solenoid handSolenoid;
    private final CANSparkMax handRotationMotor;
    private final RelativeEncoder handRotationEncoder;
    private final SparkMaxPIDController handRotationPidController;

    public Hand() {
        handSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, HandConst.Ports.HandSolenoid);

        handRotationMotor = new CANSparkMax(HandConst.Ports.HandRotationMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        handRotationMotor.setInverted(true);

        handRotationPidController = handRotationMotor.getPIDController();
        handRotationEncoder = handRotationMotor.getEncoder();
        handRotationPidController.setP(ArmConst.P_HANDR);
        handRotationPidController.setI(ArmConst.I_HANDR);
        handRotationPidController.setD(ArmConst.D_HANDR);
        handRotationPidController.setIMaxAccum(ArmConst.IMax_HANDR, 0);
    }

    @Override
    public void autonomousInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void disabledInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void testInit() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void readSensors() {
        // TODO Auto-generated method stub
        //手首が回った角度
        HandState.actualHandAngle = calculateHandAngleFromRotation(handRotationEncoder.getPosition());
    }
    /**
     * 回転数から度数への変換
     * @param rotation 変換する回転数
     * @return 変換された角度の度数
     * */
    private double calculateHandAngleFromRotation(double rotation) {
        return rotation / HandConst.HandGearRatio * 360;
    }

    /**
     *度数から回転数への変換
     * @param angle 変換する角度の度数
     * @return 変換された回転数
     */
    public double calculateRotationFromHandAngle(double angle) {
        return angle * HandConst.HandGearRatio / 360;
    }


    /** 
     * つかむ離すの運動関係のモーターを動かす
     * @param isGrabbingHand trueでつかむ。
     */
    public void grabOrReleaseControl(boolean isGrabbingHand) {
        handSolenoid.set(!isGrabbingHand);
    }

    /** 手首の回転関係のモーターを動かす */
    public void controlHandRotation(double handRotationSpeed) {
        handRotationMotor.set(handRotationSpeed);
    }
    /**
     *  pidで回転
     * @param targetAngle 目指している角度
     * */
    public void pidControlHand(double targetAngle) {
        handRotationPidController.setReference(calculateRotationFromHandAngle(targetAngle), CANSparkMax.ControlType.kPosition);
    }


    /** 物体をつかむ */
    public void grabHand() {
        grabOrReleaseControl(true);
    }

    /** 物体を離す */
    public void releaseHand() {
        grabOrReleaseControl(false);
    }

    /** 手首を回転させる */
    public void rotateHand() {
        controlHandRotation(HandConst.Speeds.HandRotationSpeed);
    }

    /** 手首を逆回転させる */
    public void invertRotateHand() {
        controlHandRotation(-HandConst.Speeds.HandRotationSpeed);
    }


    /** 手首の回転を止める */
    public void stopHand() {
         controlHandRotation(DriveConst.Speeds.Neutral);
    }
    /**
     * actual angleを入力してその数に一番近い360の倍数の数を見つけて返す
     * @param actualAngle   今の角度
     * @return actualAngle +- howFromFrom360 今の角度に一番近い360の倍数の数字
     */
    static double basicPositionCalculation(double actualAngle) {
        if((actualAngle % 360) > 180) {
            double howFarFrom360 = 360 - (actualAngle % 360);
            return actualAngle + howFarFrom360;
        }
        else {
            double howFarFrom360 = actualAngle % 360;
            return actualAngle - howFarFrom360;
        }
    }
    /** 手首を所定の位置（元の位置）に戻す*/
    public void bringBackHand() {
       pidControlHand(basicPositionCalculation(HandState.actualHandAngle));
    }
    /** 手首を所定の位置に動かす*/
    public void moveHandToSpecifiedAngle() {
        pidControlHand(HandState.targetAngle);
    }
    @Override
    public void applyState() {
        if(HandState.isResetHandPID) {
            handRotationPidController.setIAccum(0);
        }

        switch(HandState.grabHandState) {
            case s_grabHand:
                grabHand();
                break;
            case s_releaseHand:
                releaseHand();
                break;            
        }
        
        switch(HandState.rotateState) {
            case s_rightRotateHand:
                rotateHand();
                break;
            case s_leftRotateHand:
                invertRotateHand();
                break;
            case s_stopHand:
                stopHand();
                break;
            case s_turnHandBack:
                bringBackHand();
                break;
            case s_moveHandToSpecifiedAngle:
                moveHandToSpecifiedAngle();
                break;
        }

    }
}
