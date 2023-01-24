package frc.robot.subsystems;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

import edu.wpi.first.wpilibj.RobotBase;

public class ArmSubsystem extends Subsystem {
    private MotorEncoderSubsystem motor_lower_;
    private MotorEncoderSubsystem motor_upper_;

    public ArmSubsystem(Subsystem parent) throws Exception {
        super(parent, "arm");
        if (RobotBase.isSimulation()) {
            motor_lower_ = new MotorEncoderSubsystem(this, "arm-motor-lower", false);
            addChild(motor_lower_);
            motor_upper_ = new MotorEncoderSubsystem(this, "arm-motor-upper", false);
            addChild(motor_upper_);
        }
    }

    public MotorEncoderSubsystem getMotorA() {
        return motor_lower_ ;
    }

    public MotorEncoderSubsystem getMotorB() {
        return motor_upper_ ;
    }
}

