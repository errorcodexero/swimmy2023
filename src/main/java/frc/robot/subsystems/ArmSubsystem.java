package frc.robot.subsystems;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class ArmSubsystem extends Subsystem {
    private MotorEncoderSubsystem motor_a_;
    private MotorEncoderSubsystem motor_b_;

    public ArmSubsystem(Subsystem parent) throws Exception {
        super(parent, "arm");

        motor_a_ = new MotorEncoderSubsystem(this, "motor_a", false);
        addChild(motor_a_);
        motor_b_ = new MotorEncoderSubsystem(this, "motor_b", false);
        addChild(motor_b_);


    }
    
}
