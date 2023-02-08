package frc.robot.Component;

import frc.robot.State;

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

    @Override
    public void applyState() {
        // TODO Auto-generated method stub
        switch(State.handState) {
            case s_grabHand:

            case s_releaseHand:
            
        }
        
    }
    
}
