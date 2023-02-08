package frc.robot.Component;

import frc.robot.State;

public class Intake implements Component{

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
        switch(State.intakeState){
            case s_outtakeConveyor:
                break;
            case s_intakeConveyor:
                break;
            case s_stopConveyor:
                break;
        }
    }
}