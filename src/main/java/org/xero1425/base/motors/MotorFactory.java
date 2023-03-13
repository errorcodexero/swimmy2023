package org.xero1425.base.motors;

import java.util.Map;
import java.util.HashMap;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.misc.SettingsValue.SettingsType;

/// \file
/// This file contains the implementation of the MotorFactory.
/// The motor factory is used to create and initialize motors for all subsystems
/// based on data stored in the parameters file.
///

/// \brief A motor factory for the robot.
/// The motor factory creates motors based on entries in the settings file.  The
/// returned motors are high level motor classes (MotorController) that hides the
/// actual type of motor being used.  Groups of motors that operate in parallel, 
/// like multiple motors on a single side of the drivebase, are returned as a single
/// MotorController object that manages the motors as a group.  For FRC motor controllers
/// each type of motor (CTRE vs SparkMax) can have its own set of CAN IDs since the vendor
/// ID is part of the overall device identification.  However, our motor factor requires
/// that all motors have their own independent CAN ID that is unique across all types of
/// motor controllers
///
public class MotorFactory {
    private MessageLogger logger_;                              // The system wide message logger
    private ISettingsSupplier settings_;                        // The system wide settings file
    private Map<String, Map<Integer, MotorController>> motors_;              // The map of motors

    private static final String BrakeMode = "brake" ;
    private static final String CoastMode = "coast" ;

    /// \brief This method creates a new motor factory.
    /// \param logger the message logger for the robot
    /// \param settings the settings file for the robot
    public MotorFactory(MessageLogger logger, ISettingsSupplier settings) {
        logger_ = logger;
        settings_ = settings;
        motors_ = new HashMap<String, Map<Integer, MotorController>>();
        motors_.put("", new HashMap<Integer, MotorController>()) ;
    }

    /// \brief This method creates a new motor based on the settings in the settings file.
    ///
    /// \param name the base name of the motor
    /// \param id the ID of the motor in the settings file
    public MotorController createMotor(String name, String id) {
        MotorController ret = null;
        try {
            String typechk = id + ":type" ;

            if (settings_.isDefined(typechk)) {
                //
                // If the property id + 'type' exists, the motor is a single motor specified
                // at the given level of the settings file.
                //
                ret = createSingleMotor(name, id, false);
                if (ret != null) {
                    //
                    // We never set the inversion property of a motor when it is created
                    // because how we set this varied depending on whether or not a motor is a single,
                    // a leader in a group, or a follower in a group.
                    //
                    ret.setInverted(isInverted(id));
                }
            }
            else {
                //
                // If we get here, we are processing a set of motors that are controlled together.
                // These motors will be returned as a single motor controller object (MotorGroupController).
                // The first motor in this group (id = '1') will be the leader, and the remaining motors in the group
                // will follow the leader.
                //
                MotorController.NeutralMode groupmode = getNeutralMode(id);
                boolean leaderInverted = false ;
                int currentIndex = 1;
                MotorGroupController group = new MotorGroupController(name);
                ret = group;

                while (true) {
                    String motorid = id + ":" + Integer.toString(currentIndex);
                    String mtype = motorid + ":type" ;

                    if (currentIndex > 1 && !settings_.isDefined(mtype)) {
                        //
                        // For multiple motors in a motor group, if the next motor index does not
                        // exist in the settings file, we are done parsing motors.  Break out of 
                        // the loop.
                        //
                        break ;
                    }

                    //
                    // Create a new motor with the index given by currentIndex.  This means under the motors information
                    // with the name given by 'name' there will be a set of child nodes numbered from 1 - N if there are
                    // N motors in the group.  This creates the Nth motor given by currentIndex.
                    //
                    MotorController single = createSingleMotor(name + ":" + Integer.toString(currentIndex), motorid, (currentIndex == 1));
                    if (single != null) {    
                        logger_.startMessage(MessageType.Info).add("created: ").add(motorid).endMessage();
                        if (groupmode != null) {
                            // 
                            // If there is a neutral mode at the group level, assign it to
                            // each of the created motors.  This will override any group mode that is given
                            // as part of the individual motor configuration.
                            //
                            single.setNeutralMode(groupmode);
                        }

                        //
                        // See if there is an inverted settings for this motor
                        //
                        boolean v = isInverted(motorid) ;

                        if (currentIndex == 1) {
                            //
                            // Set the motor to its proper inverted state
                            //
                            single.setInverted(v);
                            leaderInverted = v ;
                            group.addLeaderMotor(single) ;
                        } else {
                            //
                            // Add a motor to the group
                            //
                            group.addFollowerMotor(single, leaderInverted, v);
                        }

                        currentIndex++;
                    } else {
                        if (currentIndex == 1) {
                            //
                            // There were no motors in the group.  Display an error in the logfile and
                            // return null indicating no motor was created.
                            //
                            errorMessage(id, "no motors found that match this id");
                            return null;
                        }
                        break;
                    }
                }
            }
        } catch (Exception ex) {
            //
            // Something threw an exception while creating a motor, print an error and put the
            // stack trace in the log file
            //
            logger_.startMessage(MessageType.Error) ;
            logger_.add("an exception was caught while creating a motor - ");
            logger_.add("name", name) ;
            logger_.add("id" , id) ;
            logger_.add(" - ") ;
            logger_.add(ex.getMessage()) ;
            logger_.endMessage();
            logger_.logStackTrace(ex.getStackTrace());
            ret = null;
        }

        return ret;
    }

    //
    // Print an error message with the motor ID given
    //
    private void errorMessage(String id, String msg) {
        logger_.startMessage(MessageType.Error);
        logger_.add("error creating motor '");
        logger_.add(id);
        logger_.add("' - ").add(msg);
        logger_.endMessage();
    }

    private MotorController getMotorController(String bus, int canid) {
        MotorController ret = null ;

        if (motors_.containsKey(bus)) {
            Map<Integer, MotorController> map = motors_.get(bus) ;
            ret = map.get(canid) ;
        }

        return ret ;
    }

    //
    // Create a single motor from the settings in the settings file
    //
    private MotorController createSingleMotor(String name, String id, boolean leader) throws BadParameterTypeException, BadMotorRequestException, MotorRequestFailedException {
            
        String typeparam = id + ":type";                // This parameter holds the type of the motor
        String canparam = id + ":canid";                // This parameter holds the CAN id of the motor
        String busparam = id + ":bus" ;                 // This parameter holds the bus name of the motor

        // Test to see if the motor has a CAN id
        boolean hasid = settings_.isDefined(canparam) && settings_.getOrNull(canparam).isInteger();

        // Test to see if the motor has a type
        boolean hastype = settings_.isDefined(typeparam) && settings_.getOrNull(typeparam).isString();

        // Test to see if the motor has a bus
        boolean hasbus = settings_.isDefined(busparam) && settings_.getOrNull(busparam).isString() ;

        if (!hasid) {
            String msg = "missing motor id '" + canparam + "' - cannot create motor" ;
            errorMessage(id, msg) ;
            return null;
        }

        if (!hastype) {
            String msg = "missing motor type '" + typeparam + "' - cannot create motor" ;
            errorMessage(id, msg);
            return null;
        }

        if (!hasbus) {
            String msg = "missing motor bus '" + busparam + "' - cannot create motor" ;
            errorMessage(id, msg);
            return null;
        }

        //
        // Get the CAN id and bus for the motor
        //
        String bus = settings_.getOrNull(busparam).getString() ;
        int canid = settings_.getOrNull(canparam).getInteger();

        MotorController dup = getMotorController(bus, canid) ;
        if (dup != null) {
            //
            // There is already a motor with this CAN id.  Signal an error.
            //
            errorMessage(id, "cannot create motor, can id is already in use on bus '" + bus + "' - motor name '" + dup.getName() + "'");
            return null;
        }

        //
        // Get the motor type from the settings file
        //
        String type = settings_.getOrNull(typeparam).getString();

        //
        // Check to be sure the bus is "" except for the TalonFX
        //
        if (!type.equals("talon-fx") && !bus.equals("")) {
            errorMessage(id, "cannot create motor, the bus value can only be non-empty for TalonFX motors") ;
            return null;
        }

        MotorController ctrl = null;

        //
        // Create the motor controller object based on its type
        //
        if (type.equals("romi")) {
            ctrl = new RomiMotorController(name, canid);
        } else if (type.equals("talon-srx")) {
            ctrl = new CTREMotorController(name, canid, CTREMotorController.MotorType.TalonSRX);
        } else if (type.equals("talon-fx")) {
            ctrl = new TalonFXMotorController(name, bus, canid, leader);
        } else if (type.equals("victor-spx")) {
            ctrl = new CTREMotorController(name, canid, CTREMotorController.MotorType.VictorSPX);
        } else if (type.equals("sparkmax-brushless")) {
            ctrl = new SparkMaxMotorController(name, canid, true, leader);
        } else if (type.equals("sparmmax-brushed")) {
            ctrl = new SparkMaxMotorController(name, canid, false, leader);
        } else {
            errorMessage(id, "motor type '" + type + "' is not a valid motor type");
            return null;
        }

        //
        // Set the motor neutral type
        //
        MotorController.NeutralMode nm = getNeutralMode(id);
        if (nm != null)
            ctrl.setNeutralMode(nm);

        double freelimit = getFreeCurrentLimit(id);
        double stalllimit = getStallCurrentLimit(id) ;
        if (!Double.isInfinite(freelimit) && !Double.isInfinite(stalllimit)) {
            ctrl.setCurrentLimit(freelimit, stalllimit);
        }      

        return ctrl ;
    }

    private double getFreeCurrentLimit(String id) throws BadParameterTypeException {
        String pname = id + ":free-current-limit";
        SettingsValue v = settings_.getOrNull(pname);
        double ret = Double.POSITIVE_INFINITY ;

        if (v != null) {
            if (!v.isDouble() && !v.isInteger()) {
                logger_.startMessage(MessageType.Error).add("parameter '").add(pname).add("'") ;
                logger_.add(" - does not have a double or integer type") ;
                throw new BadParameterTypeException(SettingsType.String, v.getType()) ;
            }

            ret = v.getDouble();
        }

        return ret;
    }

    private double getStallCurrentLimit(String id) throws BadParameterTypeException {
        String pname = id + ":stall-current-limit";
        SettingsValue v = settings_.getOrNull(pname);
        double ret = Double.POSITIVE_INFINITY ;

        if (v != null) {
            if (!v.isDouble() && !v.isInteger()) {
                logger_.startMessage(MessageType.Error).add("parameter '").add(pname).add("'") ;
                logger_.add(" - does not have a double or integer type") ;
                throw new BadParameterTypeException(SettingsType.String, v.getType()) ;
            }

            ret = v.getDouble();
        }

        return ret;
    }

    //
    // Get the neutral mode from the JSON object that describes the motor (or motor group)
    //
    private MotorController.NeutralMode getNeutralMode(String id) throws BadParameterTypeException {
        SettingsValue v ;
        String pname = id + ":neutral-mode" ;
        
        v = settings_.getOrNull(pname) ;
        if (v == null)
            return null ;

        if (!v.isString()) {
            logger_.startMessage(MessageType.Error).add("parameter '").add(pname).add("'") ;
            logger_.add(" - does not have string type") ;
            throw new BadParameterTypeException(SettingsType.String, v.getType()) ;
        }

        MotorController.NeutralMode mode ;
        if (v.getString().equals(BrakeMode))
        {
            mode = MotorController.NeutralMode.Brake ;
        }
        else if (v.getString().equals(CoastMode))
        {
            mode = MotorController.NeutralMode.Coast ;
        }
        else 
        {
            logger_.startMessage(MessageType.Warning).add("parameter '").add(pname).add("'") ;
            logger_.add(" - is a string but is not " + BrakeMode + "'' or '" + CoastMode + "'") ;
            mode = null ;
        }

        return mode ;
    }

    //
    // Get the inverted mode from the JSON object that describes the motor (or motor group)
    //
    private boolean isInverted(String id) throws BadParameterTypeException {
        SettingsValue v ;
        String pname = id + ":inverted" ;
        
        v = settings_.getOrNull(pname) ;
        if (v == null)
            return false ;

        if (!v.isBoolean()) {
            logger_.startMessage(MessageType.Error).add("parameter '").add(pname).add("'") ;
            logger_.add(" - does not have boolean type") ;
            throw new BadParameterTypeException(SettingsType.Boolean, v.getType()) ;
        }

        return v.getBoolean() ;
    }
} ;
