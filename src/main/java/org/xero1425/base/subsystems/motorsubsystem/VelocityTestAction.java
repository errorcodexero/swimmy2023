package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class VelocityTestAction extends MotorAction {
    private SimpleWidget widget_ ;
    private MotorEncoderVelocityAction action_ ;
    private double current_ ;

    public VelocityTestAction(MotorEncoderSubsystem sub) throws MissingParameterException, BadParameterTypeException, BadMotorRequestException {
        super(sub) ;

        current_ = 0.0 ;
        try {
            action_ = new MotorEncoderVelocityAction(sub, "testvelaction", current_) ;
        }
        catch(Exception ex) {
            action_ = null ;
        }
        var item = Shuffleboard.getTab("MotorTest").add("Velocity", 0.0) ;
        widget_ = item.withWidget(BuiltInWidgets.kTextView) ;
    }

    public void start() throws Exception {
        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem() ;
        sub.setAction(action_) ;
    }

    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        current_ = widget_.getEntry().getDouble(current_) ;
        action_.setTarget(current_);
    }

    public void cancel() {
        action_.cancel() ;
        getSubsystem().setAction(null) ;
    }

    public String toString(int indent) {
        return spaces(indent) + "VelocityTestAction" ;
    }
}
