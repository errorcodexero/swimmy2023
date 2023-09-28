package frc.robot.subsystems.arm;

import org.xero1425.base.LoopType;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.MotorController.NeutralMode;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class ArmSubsystem extends Subsystem {
    private MotorEncoderSubsystem motor_lower_;
    private MotorEncoderSubsystem motor_upper_;

    public ArmSubsystem(Subsystem parent) throws Exception {
        super(parent, "arm");
        
        motor_lower_ = new MotorEncoderSubsystem(this, "arm-motor-lower", false, true);
        motor_lower_.setDumpCurrents(false);        // Change to true to dump currents in log file every robot loop
        addChild(motor_lower_);
        
        motor_upper_ = new MotorEncoderSubsystem(this, "arm-motor-upper", false, true);
        motor_upper_.setDumpCurrents(false);        // Change to true to dump currents in log file every robot loop
        addChild(motor_upper_);

        motor_lower_.getMotorController().setNeutralDeadband(0.001) ;
        motor_upper_.getMotorController().setNeutralDeadband(0.001) ;
    }

    protected void computeMyState() {
    }

    public MotorEncoderSubsystem getLowerSubsystem() {
        return motor_lower_ ;
    }

    public MotorEncoderSubsystem getUpperSubsystem() {
        return motor_upper_ ;
    }

    @Override
    public void init(LoopType prev, LoopType current) {
        super.init(prev, current) ;

        if (prev == LoopType.Autonomous && current == LoopType.Disabled) {
            try {
                motor_upper_.getMotorController().setNeutralMode(NeutralMode.Brake);
                motor_lower_.getMotorController().setNeutralMode(NeutralMode.Brake);
            } catch (BadMotorRequestException | MotorRequestFailedException e) {
            }
        }
        else if (prev == LoopType.Teleop && current == LoopType.Disabled) {
            try {
                motor_upper_.getMotorController().setNeutralMode(NeutralMode.Coast);
                motor_lower_.getMotorController().setNeutralMode(NeutralMode.Coast);
            } catch (BadMotorRequestException | MotorRequestFailedException e) {
            }            
        }
    }
}

