package frc.robot.subsystems.toplevel;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.sdsswerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.SwimmyRobot2023;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.oi.Swimmy2023OISubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.Action;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;


public class Swimmy2023RobotSubsystem extends RobotSubsystem {

    //
    // The subsystems
    //
    private GPMSubsystem gpm_;
    private SDSSwerveDriveSubsystem db_;
    private Swimmy2023OISubsystem oi_;
    private LimeLightSubsystem limelight_;

    //
    // Digital IOs
    //
    DigitalOutput display_out_2_ ;
    DigitalOutput display_out_3_ ;

    //
    // The field related data
    //
    private FieldLocationData locdata_ ;

    //
    // The current operation controller
    //
    private OperationCtrl ctrl_ ;

    public Swimmy2023RobotSubsystem(XeroRobot robot) throws Exception {
        super(robot, "Swimmy2023RobotSubsystem") ;

        SwimmyRobot2023 swimmy = (SwimmyRobot2023)robot ;
        locdata_ = swimmy.getFieldData() ;

        db_ = new SDSSwerveDriveSubsystem(this, "swerve" );
        addChild(db_);

        oi_ = new Swimmy2023OISubsystem(this, db_);
        addChild(oi_);

        limelight_ = new LimeLightSubsystem(this, "limelight");
        addChild(limelight_);
        
        gpm_ = new GPMSubsystem(this);
        addChild(gpm_);     

        db_.setVision(limelight_);

        display_out_2_ = new DigitalOutput(2) ;
        display_out_3_ = new DigitalOutput(3) ;
    }

    public void setDisplayState(GamePiece st) {
        switch(st) {
            case Cone:
                display_out_2_.set(true) ;
                display_out_3_.set(false);
                break;

            case Cube:
                display_out_2_.set(false);
                display_out_3_.set(true) ;
                break; 

            default:
                display_out_2_.set(false);
                display_out_3_.set(false);
                break ;
        }
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

    public FieldLocationData getFieldData() {
        return locdata_;
    }

    private boolean isOperationValid(RobotOperation oper) {
        MessageLogger logger = getRobot().getMessageLogger();
        boolean ret = true ;

        if (oper.getGround()) {
            if (oper.getAction() != Action.Collect)
                return false;

            return true ;
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
            setDisplayState(oper.getGamePiece());
            
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
            getOI().enableGamepad();
            getSwerve().enableVision(true);
            setDisplayState(GamePiece.None);
            ctrl_ = null;

            return false;
        }

        try {
            ctrl_.start() ;
        } catch (BadParameterTypeException | MissingParameterException ex) {
            logger.startMessage(MessageType.Error) ;
            logger.add("expection caught while starting operation controller in Swimmy2023RobotSubsystem");
            logger.logStackTrace(ex.getStackTrace());
            logger.endMessage();
            getOI().enableGamepad();
            getSwerve().enableVision(true);
            setDisplayState(GamePiece.None);
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
            getOI().enableGamepad();
            getSwerve().enableVision(true);
            setDisplayState(GamePiece.None);
        }

        if (ctrl_ != null) {
            ctrl_.run() ;
            if (ctrl_.isDone()) {
                MessageLogger logger = getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Debug, getLoggerID()) ;
                logger.add("Swimmy2023RobotSubsystem: completed: " + ctrl_.getOper().toString()) ;
                logger.endMessage();
                getOI().enableGamepad();
                getSwerve().enableVision(true);
                setDisplayState(GamePiece.None);
                ctrl_ = null;
            }
        }
    }
}