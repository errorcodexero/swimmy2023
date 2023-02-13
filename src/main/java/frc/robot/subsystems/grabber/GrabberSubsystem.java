package frc.robot.subsystems.grabber;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;

public class GrabberSubsystem extends Subsystem {

    private MotorEncoderSubsystem motor_spin_ ;
    private MotorEncoderSubsystem motor_grab_ ;

    private DigitalInput sensor_bottom_ ;
    private DigitalInput sensor_top_ ;
    private boolean sensor_value_ ;

    public GrabberSubsystem(Subsystem parent) throws Exception {
        super(parent, "grabber");
        
        motor_spin_ = new MotorEncoderSubsystem(this, "grabber-motor-spin", false);
        addChild(motor_spin_);
        
        motor_grab_ = new MotorEncoderSubsystem(this, "grabber-motor-grab", false);
        addChild(motor_grab_);

        int v = getSettingsValue("hw:sensor:top").getInteger();
        sensor_top_ = new DigitalInput(v);

        v = getSettingsValue("hw:sensor:bottom").getInteger();
        sensor_bottom_ = new DigitalInput(v);
    }

    public boolean isSensorActive() {
        return sensor_value_ ;
    }

    @Override
    protected void computeMyState() {
        if (!sensor_top_.get() || !sensor_bottom_.get()) {
            sensor_value_ = true ;
        }
        else {
            sensor_value_ = false ;
        }
        putDashboard("grabber", DisplayType.Always, sensor_value_);
    }

    public MotorEncoderSubsystem getGrabMotorSubsystem() {
        return motor_grab_ ;
    }

    public MotorEncoderSubsystem getSpinMotorSubsystem() {
        return motor_spin_ ;
    }
}
