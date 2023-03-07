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
import frc.robot.subsystems.toplevel.RobotOperation.Location;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;


public class Swimmy2023RobotSubsystem extends RobotSubsystem {
    private final String OIError = "OIError" ;

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
                display_out_3_.set(true) ;
                display_out_2_.set(false);
                break;

            case Cube:
                display_out_3_.set(false);
                display_out_2_.set(true) ;
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
        boolean ret = true ;

        if (oper.getGround()) {
            //
            // Ground collect rules
            //
            if (oper.getAction() != Action.Collect) {
                putDashboard(OIError, DisplayType.Always, "ground place is not valid");
                return false;
            }

            if (oper.getGamePiece() == GamePiece.None) {
                putDashboard(OIError, DisplayType.Always, "ground operation with gamepiece 'none'");
                return false;
            }
        }
        else {
            //
            // Shelf/Loading station rules
            //
            if (oper.getAction() == Action.Collect) {
                //
                // Collect Loading Station Rules
                //
                if (oper.getGamePiece() == GamePiece.None) {
                    putDashboard(OIError, DisplayType.Always, "loading station collect with gamepiece 'none'");
                    return false ;
                }

                if (oper.getSlot() == Slot.Middle) {
                    putDashboard("OIError", DisplayType.Always, "collect operation must be slot Left or Right");
                    ret = false ;
                }
            }
            else {
                //
                // Grid Placement Rules
                // 
                if (oper.getGamePiece() == GamePiece.None) {
                    putDashboard(OIError, DisplayType.Always, "grid place with gamepiece 'none'");
                    return false ;
                }

                if (oper.getSlot() == Slot.Middle) {
                    if (oper.getGamePiece() != GamePiece.Cube && oper.getLocation() != Location.Bottom) {
                        putDashboard(OIError, DisplayType.Always, "grid place in middle slot must be gamepiece 'cube'");
                        return false ;
                    }
                }
                else {
                    if ((oper.getLocation() == Location.Top || oper.getLocation() == Location.Middle) && oper.getGamePiece() == GamePiece.Cube) {
                        putDashboard(OIError, DisplayType.Always, "grid place in left/right slot and top/middle location must be gamepiece 'cone'");
                        return false ; 
                    }
                }
            }
        }

        return ret ;
    }

    @Override
    public void computeState() {
        super.computeState();

        // LocationData loc = getLimeLight().getLocation(getSwerve().getPose());
        // if (loc == null) {
        //     putDashboard("v-x", DisplayType.Always, "NONE");
        //     putDashboard("v-y", DisplayType.Always, "NONE");
        //     putDashboard("v-h", DisplayType.Always, "NONE");
        // }
        // else {
        //     Pose2d p2d = loc.location.toPose2d() ;
        //     putDashboard("v-x", DisplayType.Always, p2d.getX());
        //     putDashboard("v-y", DisplayType.Always, p2d.getY());
        //     putDashboard("v-h", DisplayType.Always, p2d.getRotation().getDegrees());
        // }

        // putDashboard("db-x", DisplayType.Always, getSwerve().getPose().getX()) ;
        // putDashboard("db-y", DisplayType.Always, getSwerve().getPose().getY()) ;
        // putDashboard("db-h", DisplayType.Always, getSwerve().getPose().getRotation().getDegrees()) ;
    }

    public boolean isOperationComplete() {
        return ctrl_ == null ;
    }

    public OperationCtrl getRunningController() {
        return ctrl_ ;
    }

    public boolean setOperation(RobotOperation oper) {
        if (ctrl_ != null) {
            //
            // Another operation is running, we just ignore the new one
            //
            return true ;
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