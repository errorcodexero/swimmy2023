package frc.robot.subsystems;

import org.xero1425.base.subsystems.Subsystem;

public class GPMSubsystem extends Subsystem {
    private ArmSubsystem armSubsystem;
    private GrabberSubsystem grabberSubsystem;

    public GPMSubsystem(Subsystem parent) throws Exception {
        super(parent, "gpm");
        
        armSubsystem = new ArmSubsystem(this);
        addChild(armSubsystem);
        
        grabberSubsystem = new GrabberSubsystem(this);
        addChild(grabberSubsystem);

    }
    
}
