package frc.robot.subsystems.toplevel;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.sdsswerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.SwimmyRobot2023;
<<<<<<< HEAD
import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.gpm.GPMStowAction;
=======
>>>>>>> main
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.oi.Swimmy2023OISubsystem;


public class Swimmy2023RobotSubsystem extends RobotSubsystem {


    //
    // The subsystems
    //
    private GPMSubsystem gpm_;
    private SDSSwerveDriveSubsystem db_;
    private Swimmy2023OISubsystem oi_;
    private LimeLightSubsystem limelight_;

    //
<<<<<<< HEAD
    // Actions we assign to the various subsystems
    //
    private SwerveDriveToPoseAction drive_to_;
    private GPMCollectAction gpm_shelf_collect_action_ ;
    private GPMStowAction gpm_stow_action_ ;

    //
    // The current operation
    //
    
=======
    // The field related data
    //
    private FieldLocationData locdata_ ;

    //
    // The current operation controller
    //
    private OperationCtrl ctrl_ ;

>>>>>>> main
    public Swimmy2023RobotSubsystem(XeroRobot robot) throws Exception {
        super(robot, "Swimmy2023RobotSubsystem") ;

        SwimmyRobot2023 swimmy = (SwimmyRobot2023)robot ;

        db_ = new SDSSwerveDriveSubsystem(this, "swerve" );
        addChild(db_);

        oi_ = new Swimmy2023OISubsystem(this, db_);
        addChild(oi_);

        limelight_ = new LimeLightSubsystem(this, "limelight");
        addChild(limelight_);
        
        gpm_ = new GPMSubsystem(this);
        addChild(gpm_);     

        db_.setVision(limelight_);
<<<<<<< HEAD
        
        gpm_shelf_collect_action_ = new GPMCollectAction(gpm_);
        gpm_stow_action_ = new GPMStowAction(gpm_);

=======
>>>>>>> main
    }

    public GPMSubsystem getGPM() {
        return gpm_ ;
    }

    public SwerveBaseSubsystem getSwerve() {
        return db_ ;
    }

    public Swimmy2023OISubsystem getOI() {
        return oi_ ;
    }

    public LimeLightSubsystem getLimeLight() {
        return limelight_ ;
    }

<<<<<<< HEAD
    
=======
    public FieldLocationData getFieldData() {
        return locdata_;
    }

    private boolean isOperationValid(RobotOperation oper) {
        MessageLogger logger = getRobot().getMessageLogger();
        boolean ret = true ;

        if (oper.getGround()) {
            if (oper.getAction() != Action.Collect)
                return false;
        }

        if (oper.getAction() == Action.Collect) {
            //
            // Rules for collect
            //
            if (oper.getSlot() == Slot.Middle) {
                logger.startMessage(MessageType.Error);
                logger.add("Swimmy2023RobotSubsystem: invalid operation assigned - collect operation must be slot Left or Right");
                logger.endMessage();
                ret = false ;
            }
        }
        else {
            //
            // Rules for placement
            // 
        }

        return ret ;
    }

    public boolean isOperationComplete() {
        return ctrl_ == null ;
    }

    public boolean setOperation(RobotOperation oper) {
        if (ctrl_ != null) {
            //
            // If another controller is running, we cannot override it.  The gunner must press
            // abort to stop the current operation before assigning a new operation.
            //
            return false ;
        }

        if (!isOperationValid(oper)) {
            //
            // If the operation requested is invalid (like collecting from the middle slot), then
            // return false so we alert the gunner
            //
            return false ;
        }

        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("Swimmy2023RobotSubsystem: starting: " + oper.toString()) ;
        logger.endMessage();

        try {
            if (oper.getGround()) {
                ctrl_ = new AutoCollectGroundOpCtrl(this, oper);
            }
            else if (oper.getManual()) {
                if (oper.getAction() == Action.Place) {
                    ctrl_ = new ManualPlaceOpCtrl(this, oper);
                }
                else {
                    ctrl_ = new ManualCollectOpCtrl(this, oper);
                }
            }
            else {
                if (oper.getAction() == Action.Place) {
                    ctrl_ = new AutoPlaceOpCtrl(this, oper);
                }
                else {
                    ctrl_ = new AutoCollectOpCtrl(this, oper);
                }
            }
        }
        catch(Exception ex) {
            logger.startMessage(MessageType.Error) ;
            logger.add("expection caught while creating operation controller in Swimmy2023RobotSubsystem");
            logger.logStackTrace(ex.getStackTrace());
            logger.endMessage();
            ctrl_ = null;

            return false;
        }

        return true ;
    }

    public void abort() {
        if (ctrl_ != null) {
            try {
                ctrl_.abort();
            }
            catch(Exception ex) {
                MessageLogger logger = getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("expection caught while aborting operation controller in Swimmy2023RobotSubsystem");
                logger.logStackTrace(ex.getStackTrace());
                logger.endMessage();                
            }
        }
    }


    @Override
    public void run() throws Exception {
        super.run() ;

        Gamepad gp = oi_.getGamePad() ;
        if (gp != null && gp.isEnabled() == false && gp.isXPressed() && gp.isAPressed()) {
            abort() ;
            gp.enable();
        }

        if (ctrl_ != null) {
            ctrl_.run() ;
            if (ctrl_.isDone()) {
                MessageLogger logger = getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Debug, getLoggerID()) ;
                logger.add("Swimmy2023RobotSubsystem: completed: " + ctrl_.getOper().toString()) ;
                logger.endMessage();
                ctrl_ = null;
            }
        }
    }
>>>>>>> main
}