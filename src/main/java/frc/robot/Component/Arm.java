package frc.robot.Component;

import frc.robot.State;

public class Arm implements Component{

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
        switch(State.armState) {
            case s_fixArmPosition:

            case s_moveArmMotor:

            case s_moveArmToSpecifiedPosition:
        }
        // TODO Auto-generated method stub
        
    }
    
}
