package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.component.Arm;
import frc.robot.component.Component;
import frc.robot.component.Drive;
import frc.robot.component.Hand;
import frc.robot.component.Intake;
import frc.robot.phase.Autonomous;
import frc.robot.subClass.Const;
import frc.robot.subClass.ExternalSensors;
import frc.robot.subClass.MQTT;
import frc.robot.subClass.Util;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;

public class Robot extends TimedRobot {

    ArrayList<Component> components;

    ExternalSensors externalSensors;
    MQTT mqtt = new MQTT();

    PrintStream defaultConsole = System.out;
    ByteArrayOutputStream newConsole = new ByteArrayOutputStream();

    @Override
    public void robotInit() {
        System.setOut(new PrintStream(newConsole));
        Const.ConstInit();
        Thread thread = new Thread(() -> {
             mqtt.connect();
         });
        thread.start();
        components = new ArrayList<>();
        components.add(new Drive());
        components.add(new Intake());
        components.add(new Hand());
        components.add(new Arm());

        externalSensors = new ExternalSensors();

        State.StateInit();
        Util.sendSystemOut(defaultConsole, newConsole);
        defaultConsole.print(newConsole);
        newConsole = new ByteArrayOutputStream();
    }

    @Override
    public void robotPeriodic() {
        Util.sendSystemOut(defaultConsole, newConsole);
        defaultConsole.print(newConsole);
        newConsole = new ByteArrayOutputStream();
    }

    @Override
    public void autonomousInit() {
        for (Component component : components) {
            component.autonomousInit();
        }
        Autonomous.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        State.StateReset();
        externalSensors.readExternalSensors();
        for (Component component : components) {
            component.readSensors();
        }

        Autonomous.run();

        for (Component component : components) {
            component.applyState();
        }
    }

    @Override
    public void teleopInit() {
        State.mode = State.Modes.k_drive;

        for (Component component : components) {
            component.teleopInit();
        }
    }

    @Override
    public void teleopPeriodic() {
        State.StateReset();
        externalSensors.readExternalSensors();
        for (Component component : components) {
            component.readSensors();
        }

        State.mode.changeMode();

        State.mode.changeState();

        for (Component component : components) {
            component.applyState();
        }
        Util.allSendConsole();
    }

    @Override
    public void disabledInit() {
        for (Component component : components) {
            component.disabledInit();
        }
    }

    @Override
    public void disabledPeriodic() {
        externalSensors.readExternalSensors();
        for (Component component : components) {
            component.readSensors();
        }
        Util.allSendConsole();
    }

    @Override
    public void testInit() {
        State.mode = State.Modes.k_test;

        for (Component component : components) {
            component.testInit();
        }
    }

    @Override
    public void testPeriodic() {
        externalSensors.readExternalSensors();
        State.StateReset();
        for (Component component : components) {
            component.readSensors();
        }
        State.mode.changeState();

        for (Component component : components) {
            component.applyState();
        }
    }
}
