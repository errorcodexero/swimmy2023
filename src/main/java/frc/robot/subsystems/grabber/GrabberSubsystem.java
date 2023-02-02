package frc.robot.subsystems.grabber;

import org.xero1425.base.pneumatics.XeroSolenoid;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;

public class GrabberSubsystem extends Subsystem {

    private MotorEncoderSubsystem motor_left_ ;
    private MotorEncoderSubsystem motor_right_ ;
    private DigitalInput sensor_ ;
    private XeroSolenoid solenoid_ ;
    private boolean sensor_value_ ;

    public GrabberSubsystem(Subsystem parent) throws Exception {
        super(parent, "grabber");
        
        motor_left_ = new MotorEncoderSubsystem(this, "grabber-motor-left", false);
        addChild(motor_left_);
        
        motor_right_ = new MotorEncoderSubsystem(this, "grabber-motor-right", false);
        addChild(motor_right_);

        int v = getSettingsValue("hw:sensor").getInteger();
        sensor_ = new DigitalInput(v);

        solenoid_ = new XeroSolenoid(this, "solenoid") ;
    }

    public void open() {
        solenoid_.set(true);
    }

    public void close() {
        solenoid_.set(false);
    }

    @Override
    protected void computeMyState() {
        sensor_value_ = sensor_.get() ;
        putDashboard("grabber", DisplayType.Always, sensor_value_);
    }

    public MotorEncoderSubsystem getLeftSubsystem() {
        return motor_left_ ;
    }

    public MotorEncoderSubsystem getRightSubsystem() {
        return motor_right_ ;
    }
}
