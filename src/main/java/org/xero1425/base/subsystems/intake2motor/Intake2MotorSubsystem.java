package org.xero1425.base.subsystems.intake2motor;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.SettingsValue;

public class Intake2MotorSubsystem extends MotorEncoderSubsystem {
    private MotorController spinner_;
    private double spinner_power_ ;

    public Intake2MotorSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name, false); // Motor 1, in the base class

        // Motor 2, explicitly create it
        String motorname = "subsystems:" + name + ":hw:spinner:motor" ;
        spinner_ = getRobot().getMotorFactory().createMotor("intake-spinner", motorname);
        this.setAction(new IntakePositionPowerAction(this, "collect:offpos", "collector:offpower", true, true));
    }

    public void setSpinnerPower(double p) throws BadMotorRequestException, MotorRequestFailedException {
        spinner_power_ = p ;
        spinner_.set(p);
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("spinner-power")) {
            v = new SettingsValue(spinner_power_) ;
        }

        return v ;
    }
}
