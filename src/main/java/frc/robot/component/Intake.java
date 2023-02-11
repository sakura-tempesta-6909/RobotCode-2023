package frc.robot.component;

import frc.robot.State;
import frc.robot.subClass.Const;

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

    /** conveyor関係のモーターを動かす */
    public void conveyorControl(double rollerSpeed) {

    }

    /** CONEとCUBEを出す */
    public void outtakeConveyor() {
        conveyorControl(Const.Speeds.IntakeSpeed);
    }

    /** CONEとCUBEを回収する */
    public void intakeConveyor() {
        conveyorControl(Const.Speeds.OuttakeSpeed);
    }

    /** Conveyorを動かさない */
    public void stopConveyor() {
        conveyorControl(Const.Speeds.Neutral);
    }

    @Override
    public void applyState() {
        switch(State.intakeState){
            case s_outtakeConveyor:
                outtakeConveyor();
                break;
            case s_intakeConveyor:
                intakeConveyor();
                break;
            case s_stopConveyor:
                stopConveyor();
                break;
        }
    }
}