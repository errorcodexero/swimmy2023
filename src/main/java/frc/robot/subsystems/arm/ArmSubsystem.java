package frc.robot.subsystems.arm;

import org.xero1425.base.LoopType;
import org.xero1425.base.motors.MotorController.NeutralMode;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class ArmSubsystem extends Subsystem {
    private MotorEncoderSubsystem motor_lower_;
    private MotorEncoderSubsystem motor_upper_;

    public ArmSubsystem(Subsystem parent) throws Exception {
        super(parent, "arm");
        
        motor_lower_ = new MotorEncoderSubsystem(this, "arm-motor-lower", false, true);
        addChild(motor_lower_);
        
        motor_upper_ = new MotorEncoderSubsystem(this, "arm-motor-upper", false, true);
        addChild(motor_upper_);
    }

    public MotorEncoderSubsystem getLowerSubsystem() {
        return motor_lower_ ;
    }

    public MotorEncoderSubsystem getUpperSubsystem() {
        return motor_upper_ ;
    }

    @Override
    public void init(LoopType ltype) {
        super.init(ltype) ;

        if (ltype == LoopType.Disabled) {
            try {
                motor_upper_.getMotorController().setNeutralMode(NeutralMode.Coast);
            }
            catch(Exception ex) {
            }
        }
    }
}

