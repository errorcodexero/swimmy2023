package frc.robot.subsystems;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

import edu.wpi.first.wpilibj.RobotBase;

public class ArmSubsystem extends Subsystem {
    private MotorEncoderSubsystem motor_a_;
    private MotorEncoderSubsystem motor_b_;

    public ArmSubsystem(Subsystem parent) throws Exception {
        super(parent, "arm");
        if (RobotBase.isSimulation()) {
            motor_a_ = new MotorEncoderSubsystem(this, "arm-motor-a", false);
            addChild(motor_a_);
            motor_b_ = new MotorEncoderSubsystem(this, "arm-motor-b", false);
            addChild(motor_b_);
        }
    }
}
