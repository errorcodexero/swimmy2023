package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DriverStation;

public class PowerTestAction extends MotorAction {
    private double current_ ;

    public PowerTestAction(MotorEncoderSubsystem sub) throws MissingParameterException, BadParameterTypeException, BadMotorRequestException {
        super(sub) ;

        current_ = 0.0 ;
    }

    public void start() throws Exception {
        super.start() ;
        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem() ;
        sub.setPower(current_) ;
    }

    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        // Left X axis
        double newpower = DriverStation.getStickAxis(0, 0) ;
        if (Math.abs(newpower - current_) > 0.01) {
            current_ = newpower ;
            getSubsystem().setPower(current_);
        }
    }

    public void cancel() {
        super.cancel() ;
        getSubsystem().setPower(0.0) ;
    }

    public String toString(int indent) {
        return spaces(indent) + "PowerTestAction" ;
    }
}
