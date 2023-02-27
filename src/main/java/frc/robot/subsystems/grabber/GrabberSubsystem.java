package frc.robot.subsystems.grabber;

import org.xero1425.base.LoopType;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;

public class GrabberSubsystem extends Subsystem {

    private MotorEncoderSubsystem motor_grab_ ;
    private MotorEncoderSubsystem motor_spin_ ;
    private DigitalInput sensor_upper_ ;
    private DigitalInput sensor_lower_ ;
    private boolean sensor_upper_value_ ;
    private boolean sensor_lower_value_ ;

    public GrabberSubsystem(Subsystem parent) throws Exception {
        super(parent, "grabber");
        
        motor_grab_ = new MotorEncoderSubsystem(this, "grabber-motor-grab", false);
        motor_grab_.getMotorController().setCurrentLimit(40.0, 20.0);
        addChild(motor_grab_);
        
        motor_spin_ = new MotorEncoderSubsystem(this, "grabber-motor-spin", false);
        motor_spin_.getMotorController().setCurrentLimit(40.0, 20.0);
        addChild(motor_spin_);

        int v = getSettingsValue("hw:sensor:lower").getInteger();
        sensor_lower_ = new DigitalInput(v);
        v = getSettingsValue("hw:sensor:upper").getInteger();
        sensor_upper_ = new DigitalInput(v);
    }

    @Override
    public void init(LoopType lt) {
    }

    @Override
    protected void computeMyState() {
        sensor_upper_value_ = !sensor_upper_.get() ;
        sensor_lower_value_ = !sensor_lower_.get() ;
        putDashboard("s-upper", DisplayType.Always, sensor_upper_value_);
        putDashboard("s-lower", DisplayType.Always, sensor_lower_value_);
    }

    public boolean sensor() {
        return sensor_upper_value_ || sensor_lower_value_;
    }

    public MotorEncoderSubsystem getGrabSubsystem() {
        return motor_grab_ ;
    }

    public MotorEncoderSubsystem getSpinSubsystem() {
        return motor_spin_ ;
    }
}
