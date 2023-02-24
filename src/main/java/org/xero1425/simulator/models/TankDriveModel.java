package org.xero1425.simulator.models;

import org.xero1425.simulator.engine.SimulationModel;

import edu.wpi.first.hal.simulation.EncoderDataJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.misc.XeroMath;


//
// The model for a tank drive base.  This model also includes publishing of state to thet network tables
// which is not required for a model and is only useful if any visualation of the simulation supports visualizing
// the network table information.  Visualiation of simulations is not complete yet and is an experimental feature
// at the present.
//

public class TankDriveModel extends SimulationModel {
    //
    // The network table name for publising information about tank drive in the network tables
    //
    private final static String SubTableName = "tankdrive" ;

    //
    // The name of the event that contains the x position of the robot.  This is also the name
    // of the value that is published in the network table for the X position of the robot.
    //
    private final static String TankDriveXPos = "xpos" ;

    //
    // The name of the event that contains the y position of the robot.   This is also the name
    // of the value that is published in the network table for the Y position of the robot.
    //
    private final static String TankDriveYPos = "ypos" ;

    //
    // The name of the event that contains the angle of the robot.  This is also the name
    // of the value that is published in the network table for the angle of the robot.
    //
    private final static String TankDriveAngle = "angle" ;

    //
    // The name of the network table entry storing status text for the robot.
    //
    private final static String TankDriveText = "text" ;

    //
    // The model name of the model that provides status information for the visualization
    //
    private final static String TextProviderModel = "text_provider_model" ;

    //
    // The instance name of the model instance that provides status information for the visualization
    //
    private final static String TextProviderInst = "text_provider_instance" ;

    //
    // The subsystem that provides status text for visualation.  This is published as part of the
    // tank drive information in the network tables
    //
    private SimulationModel text_provider_ ;

    //
    // The simulated motor controller for the left side of the drivebase
    //
    private SimMotorController left_ ;

    //
    // The simulated motor controller for the right side of the drivebase
    //
    private SimMotorController right_ ;

    //
    // The simulated navx which much changle angle state as the robot moves
    //
    private NavXModel navx_ ;

    //
    // The diameter of the wheels
    //
    private double diameter_ ;

    //
    // The width of the robot
    //
    private double width_ ;

    //
    // The length of the robot
    //
    private double length_ ;

    //
    // The scrub factor for robot turns.  This is the deviation factor for a robot angle change based on
    // a difference is distance of the left and right wheel travel.  This deviation accounts for the friction of
    // the wheels on the floor making the turn less than ideal.
    //
    private double scrub_ ;

    //
    // The ticks output per revolution for the encoders
    //
    private double inches_per_tick_ ;

    //
    // The maximum velocity of the robot
    //
    private double max_velocity_ ;

    //
    // The maximum acceleration of the robot
    //
    private double max_accel_ ;

    //
    // The multiplier for the distance calculated.  These should be either
    // 1.0 or -1.0 and just set the direction for the given side of the robot
    //
    private double left_motor_mult_ ;
    private double right_motor_mult_ ;
    
    //
    // The current speed of the motors in revolutions per second
    //
    private double current_left_rps_ ;
    private double current_right_rps_ ;

    //
    // The current position of the left and right sides of the robot
    //
    private double left_pos_ ;
    private double right_pos_ ;

    //
    // The current angle.  This will always be between -180 and 180
    //
    private double angle_ ;

    //
    // The angle the last simulator loop.  This will always be between -180 and 180
    //
    private double last_angle_ ;

    //
    // The total angle traveled.  This will continue to beyond +/- 180
    //
    private double total_angle_ ;

    //
    // The speed of the robot, this is relative to the center of the robot.
    //
    private double speed_ ;

    //
    // The current power
    //
    private double left_power_ ;
    private double right_power_ ;

    //
    // The encoder values
    //
    private int left_enc_value_ ;
    private int right_enc_value_ ;

    //
    // The multiplier for the encoder values.  These should be either 1, or -1
    // and indicate the direction of the encoder with respect to the forward direction
    // of the robot
    //
    private int left_encoder_mult_ ;
    private int right_encoder_mult_ ;

    //
    // The revolutions traveled per power per second.
    //
    private double left_rps_per_power_per_time_ ;
    private double right_rps_per_power_per_time_ ;

    //
    // The maximum allowed RPS speed change.  Provided to prevent the model
    // from having infinite acceleration.
    //
    private double max_change_ ;

    //
    // The current position of the center of the robot
    //
    private double xpos_ ;
    private double ypos_ ;

    //
    // The position of the center of the robot last simulator loop
    //
    private double last_xpos_ ;
    private double last_ypos_ ;

    //
    // The external encoder indexes, if -1, then encoders are in the motor
    //
    private int left_encoder_index_ ;
    private int right_encoder_index_ ;

    /// \brief create a simulation model for a tank drive
    /// \param engine the simulation engine
    /// \param model the name of the model
    /// \param inst the name of the instance being created
    public TankDriveModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        navx_ = null;
        text_provider_ = null ;

        xpos_ = 0.0 ;
        ypos_ = 0.0 ;
        angle_ = 0.0 ;

        left_encoder_index_ = -1 ;
        right_encoder_index_ = -1 ;
    }

    public double getLeftPower() {
        return left_power_ ;
    }

    public double getRightPower() {
        return right_power_ ;
    }
    
    /// \brief called once at the end of the simulator loop
    @Override
    public void endCycle() {
        //
        // Write information to the log file about the position of the robot as seen by the model
        //
        MessageLogger logger = getEngine().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("tankdrive") ;
        logger.add(" ").add(xpos_) ;
        logger.add(" ").add(ypos_) ;
        logger.add(" ").add(angle_) ;
        logger.endMessage() ;

        //
        // Write information to the network tables about the position on the robot
        //
        NetworkTable table_ = NetworkTableInstance.getDefault().getTable(SimulationEngine.NetworkTableName).getSubTable(SubTableName) ;
        table_.getEntry(TankDriveXPos).setNumber(xpos_) ;
        table_.getEntry(TankDriveYPos).setNumber(ypos_) ;
        table_.getEntry(TankDriveAngle).setNumber(Math.toDegrees(angle_)) ;
        if (text_provider_ != null)
            table_.getEntry(TankDriveText).setString(text_provider_.statusString()) ;
    }

    /// \brief create a new simulation model for a tank drive.
    /// Model creation is a two step process. The constructor does very basic variable initialization.  This
    /// method completes the model creation process.  This method can rely on all of the other models required being in
    /// place.  However, the create() function on other models may or may not have been called.
    public boolean create() {
        MessageLogger logger = getEngine().getMessageLogger() ;

        //
        // If the text provider properties have been provided in the robot.json file, then look up
        // the model that will provide status text for the robot.
        //
        if (hasProperty(TextProviderModel) && hasProperty(TextProviderInst)) {
            SettingsValue modelprop = getProperty(TextProviderModel) ;
            SettingsValue instprop = getProperty(TextProviderInst) ;

            if (!modelprop.isString()) {
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" property ").addQuoted(TextProviderModel).add(" is not a string") ;
                logger.endMessage();
                modelprop = null ;
            }

            if (!instprop.isString()) {
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" property ").addQuoted(TextProviderInst).add(" is not a string") ;
                logger.endMessage();
                instprop = null ;
            }        

            if (modelprop != null && instprop != null) {
                try {
                    text_provider_ = getEngine().findModel(modelprop.getString(), instprop.getString());
                } catch (BadParameterTypeException e) {
                }

                if (text_provider_ == null) {
                    logger.startMessage(MessageType.Error);
                    logger.add("event: model ").addQuoted(getModelName());
                    logger.add(" instance ").addQuoted(getInstanceName());
                    logger.add(" the reference model for text provider does not exist") ;
                    logger.endMessage();                    
                }
            }
        }

        //
        // Attach to the motors on the left side of the robot
        //
        left_ = new SimMotorController(this, "left");
        if (!left_.createMotor())
            return false ;

        //
        // Attach to the motors on the right side of the robot
        //
        right_ = new SimMotorController(this, "right");
        if (!right_.createMotor())
            return false ;

        //
        // Attach to the navx model to update the navx angle settings as the robot turns
        //
        if (hasProperty("navx:model") && getProperty("navx:model").isString() && hasProperty("navx:instance")
                && getProperty("navx:instance").isString()) {

            String navx_model = null ;
            String navx_inst = null ;

            try {
                navx_model = getProperty("navx:model").getString();
                navx_inst = getProperty("navx:instance").getString() ;                
            } catch (BadParameterTypeException e) {
            }

            SimulationModel model = getEngine().findModel(navx_model, navx_inst) ;
            if (model != null && (model instanceof NavXModel))
                navx_ = (NavXModel)model ;
        }

        //
        // If the left:motor:inverted property exists and is true, then invert the left motors
        //
        if (hasProperty("left:motor:inverted"))
        {
            SettingsValue v = getProperty("left:motor:interted") ;
            try {
                if (v.isBoolean() && v.getBoolean())
                    left_motor_mult_ = -1.0;
            } catch (BadParameterTypeException e) {
            }
        }

        //
        // If the right:motor:inverted property exists and is true, then invert the right motors
        //
        if (hasProperty("right:motor:inverted"))
        {
            SettingsValue v = getProperty("right:motor:inverted") ;
            try {
                if (v.isBoolean() && v.getBoolean())
                    right_motor_mult_ = -1.0;
            } catch (BadParameterTypeException e) {
            }
        }

        if (hasProperty("left:encoder:index") || hasProperty("right:encoder:index"))
        {
            if (!hasProperty("left:encoder:index"))
            {
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" the model parameters include 'right:encoder:index' but not 'left:encoder:index'") ;
                logger.endMessage();

                return false ;
            }

            if (!hasProperty("left:encoder:index"))
            {
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" the model parameters include 'left:encoder:index' but not 'right:encoder:index'") ;
                logger.endMessage();

                return false ;
            }

            SettingsValue v = getProperty("left:encoder:index") ;
            try {
                left_encoder_index_ = v.getInteger() ;
            }
            catch(BadParameterTypeException ex) {
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" the model parameters include 'left:encoder:index' existed, but was not an integer") ;
                logger.endMessage();

                return false ;                
            }

            try {
                right_encoder_index_ = v.getInteger() ;
            }
            catch(BadParameterTypeException ex) {
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" the model parameters include 'right:encoder:index' existed, but was not an integer") ;
                logger.endMessage();

                return false ;                
            }
        }

        //
        // If the left:encoder:inverted property exists and is true, then invert the left encoders
        //
        if (hasProperty("left:encoder:inverted"))
        {
            SettingsValue v = getProperty("left:encoder:inverted") ;
            try {
                if (v.isBoolean() && v.getBoolean())
                    left_encoder_mult_ = -1;
            } catch (BadParameterTypeException e) {
            }
        }        

        //
        // If the right:encoder:inverted property exists and is true, then invert the right encoders
        //
        if (hasProperty("right:encoder:inverted"))
        {
            SettingsValue v = getProperty("right:encoder:inverted") ;
            try {
                if (v.isBoolean() && v.getBoolean())
                    right_encoder_mult_ = -1;
            } catch (BadParameterTypeException e) {
            }
        }         
        
        //
        // Grab the properties for the physical characteristics of the robot.  If any
        // properties do not exist or are of the wrong type, this is a problem.  We catch the
        // exception and return false indicated that model creation failed.  This will fail
        // the simulation.
        //
        try {
            diameter_ = getProperty("diameter").getDouble();
            width_ = getProperty("width").getDouble() ;
            length_ = getProperty("length").getDouble() ;
            scrub_ = getProperty("scrub").getDouble() ;
            inches_per_tick_ = getProperty("inches_per_tick").getDouble() ;
            max_velocity_ = getProperty("maxvelocity").getDouble() ;
            max_accel_ = getProperty("maxacceleration").getDouble() ;
        } catch (Exception e) {
            return false ;
        }

        //
        // Compute the RPS values based on the characteristics of the robot
        //
        double circum = diameter_ * Math.PI ;
        left_rps_per_power_per_time_ = max_velocity_ / circum ;
        right_rps_per_power_per_time_ = max_velocity_ / circum ;

        //
        // Compute the max change based on acceleration
        //
        max_change_ = max_accel_ / circum ;

        //
        // Initialize the robot position
        //
        left_motor_mult_ = 1.0 ;
        right_motor_mult_ = 1.0 ;
        current_left_rps_ = 0.0 ;
        current_right_rps_ = 0.0 ;
        left_pos_ = 0.0 ;
        right_pos_ = 0.0 ;
        angle_ = 0.0 ;
        last_angle_ = 0.0 ;
        total_angle_ = 0.0 ;

        left_encoder_mult_ = 1 ;
        right_encoder_mult_ = 1 ;

        // Set the created flag and return success
        setCreated();
        return true ;
    }

    /// \brief return the robot pose
    /// \returns the robot pose
    public Pose2d getPose() {
        return new Pose2d(xpos_, ypos_, new Rotation2d(angle_)) ;
    }

    /// \brief return the robot x position
    /// \returns the robot x position
    public double getXPos() {
        return xpos_ ;
    }

    /// \brief return the robot y position
    /// \returns the robot y position    
    public double getYPos() {
        return ypos_ ;
    }

    /// \brief return the robot angle
    /// \returns the robot angle        
    public double getAngle() {
        return Math.toDegrees(angle_) ;
    }

    /// \brief return the speed of the robot
    /// \return the speed of the robot
    public double getSpeed() {
        return speed_ ;
    }

    /// \brief return the width of the robot
    /// \return the width of the robot
    public double getWidth() {
        return width_ ;
    }

    /// \brief return the length of the robot
    /// \return the length of the robot
    public double getLength() {
        return length_ ;
    }    

    /// \brief run on simlator loop
    /// \param dt the amount of time that has passed since the last simulator loop
    public void run(double dt) {

        //
        // Get the power from the motors
        //
        left_power_ = left_.getPower() ;
        right_power_ = right_.getPower() ;

        //
        // Calculated the desired left and right revolutions per second based on the motor power
        //
        double desired_left_rps = left_rps_per_power_per_time_ * left_power_ * left_motor_mult_ ;
        double desired_right_rps = right_rps_per_power_per_time_ * right_power_ * right_motor_mult_ ;

        //
        // Calculate the actual left and right revolutions per second based on the maximum allows acceleration
        // of the robot.
        //
        current_left_rps_ = capVelocity(current_left_rps_, desired_left_rps) ;
        current_right_rps_ = capVelocity(current_right_rps_, desired_right_rps) ;

        //
        // Calculate the left and right distance traveled this robot loop
        //
        double dleft = current_left_rps_ * dt * diameter_ * Math.PI ;
        double dright = current_right_rps_ * dt * diameter_ * Math.PI ;

        //
        // Update the left and right position of the robot
        //
        left_pos_ += dleft ;
        right_pos_ += dright ;

        //
        // Update the angle of the robot based on the travel of the robot
        //
        double lrevs = left_pos_ / (Math.PI * diameter_) ;
        double rrevs = right_pos_ / (Math.PI * diameter_) ;
        double dv = (dright - dleft) / 2 * scrub_ ;
        angle_ = XeroMath.normalizeAngleRadians(angle_ + (dv * 2.0) / width_) ;
        updatePosition(dleft, dright, angle_) ;

        //
        // Calculate the speed of the robot
        //
        double distsq = (xpos_ - last_xpos_) * (xpos_ - last_xpos_) + (ypos_ - last_ypos_) * (ypos_ - last_ypos_) ;
        double dist = Math.sqrt(distsq) ;
        speed_ = dist / dt ;

        last_xpos_ = xpos_ ;
        last_ypos_ = ypos_ ;

        //
        // Compute the encoder ticks based on the position of the left and right
        // sides of the robot
        //
        left_enc_value_ = (int)(lrevs * diameter_ * Math.PI * left_encoder_mult_ / inches_per_tick_) ;
        right_enc_value_ = (int)(rrevs * diameter_ * Math.PI * right_encoder_mult_ / inches_per_tick_) ;

        if (left_encoder_index_ != -1 && right_encoder_index_ != -1) {
            EncoderDataJNI.setCount(left_encoder_index_, left_enc_value_) ;
            EncoderDataJNI.setCount(right_encoder_index_, right_enc_value_) ;
        }
        else {
            if (left_.usesTicks()) {
                left_.setEncoder(left_enc_value_);
                right_.setEncoder(right_enc_value_);
            }
            else {
                left_.setEncoder(lrevs * left_encoder_mult_);
                right_.setEncoder(rrevs * right_encoder_mult_) ;
            }
        }

        //
        // Set the navx angle based on the robot angle.
        //
        double deg = XeroMath.normalizeAngleDegrees(-Math.toDegrees(angle_)) ;
        if (navx_ != null) {
            navx_.setYaw(deg);
            navx_.setTotalAngle(Math.toDegrees(total_angle_));
        }

        MessageLogger logger = getEngine().getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("lp", left_power_) ;
        logger.add("rp", right_power_) ;
        logger.add("lrps", current_left_rps_) ;
        logger.add("rrps", current_right_rps_) ;
        logger.add("dl", dleft) ;
        logger.add("dr", dright) ;
        logger.add("lpos", left_pos_) ;
        logger.add("rpos", right_pos_) ;
        logger.add("lrevs", lrevs) ;
        logger.add("rrevs", rrevs) ;
        logger.add("lenc" , left_enc_value_) ;
        logger.add("renc", right_enc_value_) ;
        logger.endMessage();
    }

    /// \brief process an event assigned to the subsystem
    /// This subsystem understands the "xpos", "ypos", and "angle" events which
    /// allow the simulation stimulus file to set the position of the robot.  While not
    /// required, these events are usually assigned at time t = 0 to set the initial
    /// position of the robot.  If these events are assigned after t = 0, the position
    /// tracking code in this model may produce strange results.
    /// \param name the name of the event
    /// \param value the value of the event
    public boolean processEvent(String name, SettingsValue value) {
        if (name.equals(TankDriveXPos)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true ;
            }

            try {
                xpos_ = value.getDouble();
            } catch (BadParameterTypeException e) {
            }
        }
        else if (name.equals(TankDriveYPos)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true ;
            }

            try {
                ypos_ = value.getDouble();
            } catch (BadParameterTypeException e) {
            }
        }  
        else if (name.equals(TankDriveAngle)) {
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
                return true ;
            }

            try {
                angle_ = Math.toDegrees(value.getDouble()) ;
            } catch (BadParameterTypeException e) {
            }
        }               
        return true ;
    }

    //
    // Update the angle of the robot given the left and right positions of the
    // robot.
    //
    private void updatePosition(double dleft, double dright, double angle) {
        if (Math.abs(dleft - dright) < 1e-6) {
            xpos_ += dleft * Math.cos(angle) ;
            ypos_ += dright * Math.sin(angle) ;
        }
        else {
            double r = width_ * (dleft + dright) / (2 * (dright - dleft)) ;
            double wd = (dright - dleft) / width_ ;
            xpos_ = xpos_ + r * Math.sin(wd + angle) - r * Math.sin(angle) ;
            ypos_ = ypos_ - r * Math.cos(wd + angle) + r * Math.cos(angle) ;
        }

        double dangle = XeroMath.normalizeAngleRadians(angle_ - last_angle_) ;
        total_angle_ += dangle ;
    }

    //
    // Cap the velocity so that it does not exceed the maximum allowed change in a 
    // single simlator loop
    //
    private double capVelocity(double prev, double target) {
        double ret = 0.0 ;

        if (target > prev) {
            if (target > prev + max_change_)
                ret = prev +  max_change_ ;
            else
                ret = target ;
        } else {
            if (target < prev - max_change_)
                ret = prev - max_change_ ;
            else
                ret = target ;
        }

        return ret ;
    }
}