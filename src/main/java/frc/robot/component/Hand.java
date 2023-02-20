package frc.robot.component;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.State;
import frc.robot.subClass.Const;

public class Hand implements Component{
    private Solenoid handSolenoid;

    public Hand() {
        handSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Const.Ports.HandSolenoid);
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
        
        switch(State.rotateHandState) {
            case s_rotateHand:
                rotateHand();
                break;
            case s_stopHand:
                stopHand();
                break;
        }
    }
}
