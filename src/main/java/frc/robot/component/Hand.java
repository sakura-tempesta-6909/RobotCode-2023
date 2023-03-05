package frc.robot.component;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.State;
import frc.robot.subClass.Const;


public class Hand implements Component{
    private Solenoid handSolenoid;
    private final CANSparkMax handRotationMotor;
    private final RelativeEncoder handRotationEncoder;
    private final SparkMaxPIDController handRotationPidController;

    public Hand() {
        handSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Const.Ports.HandSolenoid);

        handRotationMotor = new CANSparkMax(Const.Ports.HandRotationMotor, CANSparkMaxLowLevel.MotorType.kBrushless);

        handRotationPidController = handRotationMotor.getPIDController();
        handRotationEncoder = handRotationMotor.getEncoder();
        handRotationPidController.setP(Const.Arm.P_HANDR);
        handRotationPidController.setI(Const.Arm.I_HANDR);
        handRotationPidController.setD(Const.Arm.D_HANDR);
        handRotationPidController.setIMaxAccum(Const.Arm.IMax_HANDR, 0);
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
        State.Hand.actualHandAngle = calculateHandAngleFromRotation(handRotationEncoder.getPosition());
    }
    /** 回転数から度数への変換 */
    private double calculateHandAngleFromRotation(double rotation) {
        return rotation / Const.Hand.HandGearRatio * 360;
    }

    public double calculateRotationFromHandAngle(double angle) {
        return angle * Const.Hand.HandGearRatio / 360;
    }


    /** 
     * つかむ離すの運動関係のモーターを動かす
     * @param isGrabbingHand trueかfalseでつかむ。まだ分からない。
     */
    public void grabOrReleaseControl(boolean isGrabbingHand) {
        handSolenoid.set(isGrabbingHand);
    }

    /** 手首の回転関係のモーターを動かす */
    public void controlHandRotation(double handRotationSpeed) {
        handRotationMotor.set(handRotationSpeed);
    }
    /** pidで回転*/
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
        controlHandRotation(Const.Speeds.HandRotationSpeed);
    }

    /** 手首の回転を止める */
    public void stopHand() {
        controlHandRotation(Const.Speeds.Neutral);
    }
    /** actual angleを入力してその数に一番近い360の倍数の数を見つけて返す */
    static double basicPositionCalculation(double n) {

        if((n % 360) > 180) {
            double x = 360 - (n % 360);
            return n + x;
        }
        else {
            double x = n % 360;
            return n - x;
        }
    }
    /** 手首を所定の位置（元の位置）に戻す*/
    public void bringBackHand() {
       pidControlHand(basicPositionCalculation(State.Hand.actualHandAngle));
    }
    /** 手首を所定の位置に動かす*/
    public void moveHandToSpecifiedAngle() {
        pidControlHand(State.Hand.targetAngle);
    }
    @Override
    public void applyState() {
        switch(State.grabHandState) {
            case s_grabHand:
                grabHand();
                break;
            case s_releaseHand:
                releaseHand();
                break;            
        }
        
        switch(State.Hand.rotateState) {
            case s_rotateHand:
                rotateHand();
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
