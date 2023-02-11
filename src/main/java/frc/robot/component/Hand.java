package frc.robot.component;

import frc.robot.State;
import frc.robot.subClass.Const;

public class Hand implements Component{

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

    /** つかむ離すの運動関係のモーターを動かす */
    public void fistControl(boolean grabOrRelease) {

    }

    /** 手首の回転関係のモーターを動かす */
    public void controlHandRotation(double handRotationSpeed) {

    }
    
    /** 物体をつかむ */
    public void grabHand() {
        fistControl(true);
    }

    /** 物体を離す */
    public void releaseHand() {
        fistControl(false);
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
