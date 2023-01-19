package org.xero1425.base.subsystems.oi ;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController ;
import edu.wpi.first.wpilibj.DriverStation ;

/// \file

/// \brief a base class for a gamepad.  This class provides access to the various axis, buttons, and POV
/// switches.  It does not provide any mapping to the drivebase.  This is done by derived classes.
public abstract class Gamepad extends OIDevice
{
    // The XBOX Controller attached
    private XboxController controller_ ;

    // The start time for a rumble functgion
    private double start_ ;

    // The duration of the rumble function
    private double duration_ ;

    // If true, we are rumbling
    private boolean rumbling_ ;
    
    /// \brief Create the new gamepad
    /// \param oi the OI subsystem that owns this device
    /// \param name the name of this device
    /// \param index the index used to access this device in the WPILib.
    public Gamepad(OISubsystem oi, String name, int index) {
        super(oi, name, index) ;

        controller_ = new XboxController(index) ;
        rumbling_ = false ;
    }

    /// \brief Rumble the gamepad for a fixed magnitude and duraction
    /// \param amount the magnitude of the rumble
    /// \param duration the duration of the rumble
    public void rumble(double amount, double duration) {
        controller_.setRumble(GenericHID.RumbleType.kRightRumble, amount);
        controller_.setRumble(GenericHID.RumbleType.kLeftRumble, amount) ;
        start_ = getSubsystem().getRobot().getTime() ;
        duration_ = duration ;
        rumbling_ = true ;
    }

    /// \brief Compute the state of this Gamepad device.  For the gamepad the
    /// rumble function is processed.
    @Override
    public void computeState() {
        if (rumbling_ && getSubsystem().getRobot().getTime() - start_ > duration_)
        {
            controller_.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0) ;
            controller_.setRumble(GenericHID.RumbleType.kRightRumble, 0.0) ;
            rumbling_ = false ;
        }
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed
    public boolean isRTriggerPressed() {
        boolean ret ;
        
        try {
            ret = DriverStation.getStickAxis(getIndex(), AxisNumber.RTRIGGER.value) > 0.5 ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isLTriggerPressed() {
        boolean ret ;
        
        try {
            ret = DriverStation.getStickAxis(getIndex(), AxisNumber.LTRIGGER.value) > 0.5 ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }    

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isAPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.A.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isBPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.B.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isXPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.X.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isYPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.Y.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isLJoyButtonPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.L_JOY.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isRJoyButtonPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.R_JOY.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isRBackButtonPressed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.RB.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;    
    }

    /// \brief Returns true if the right trigger is pressed
    /// \returns true if the right trigger is pressed    
    public boolean isLBackButtonPrssed() {
        boolean ret ;

        try {
            ret = DriverStation.getStickButton(getIndex(), ButtonNumber.LB.value) ;
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;       
    }

    /// \brief Returns the POV angle for the gamepad
    /// \returns the POV angle for the gamepad
    public POVAngle getPOVAngle() {
        int povval ;
        
        try {
            povval = DriverStation.getStickPOV(getIndex(), 0) ;
        }
        catch(Exception ex) {
            povval = -1 ;
        }
        
        return POVAngle.fromInt(povval) ;
    }

    /// \brief The axis numbers on the joystick
    protected enum AxisNumber {
        LEFTX(0),              ///< Left X axis
        LEFTY(1),              ///< Left Y axis
        LTRIGGER(2),           ///< Left Trigger Axis
        RTRIGGER(3),           ///< Right Trigger Axis
        RIGHTX(4),             ///< Right X axis
        RIGHTY(5) ;            ///< Right Y axis

        /// \brief the value of the axis enum
        public final int value ;
        
        private AxisNumber(int value) {
            this.value = value ;
        }
    } ;

    /// \brief buttons on the gamepad
    protected enum ButtonNumber {
        A(1),                  ///< A button
        B(2),                  ///< B button
        X(3),                  ///< X button
        Y(4),                  ///< Y button
        LB(5),                 ///< Left back button
        RB(6),                 ///< Right back button
        BACK(7),               ///< Back button
        START(8),              ///< Start button
        L_JOY(9),              ///< Left joystick button
        R_JOY(10);             ///< Right joystick button

        /// The value of the enum
        public final int value ;

        private ButtonNumber(int value) {
            this.value = value ;
        }        
    } ;

    /// \brief POV angles
    protected enum POVAngle {
        UP(0),                 ///< Up, 0 degrees
        UPRIGHT(45),           ///< UpRight, 45 degrees
        RIGHT(90),             ///< Right, 90 degrees
        DOWNRIGHT(135),        ///< DownRight, 135 degrees
        DOWN(180),               ///< Down, 180 degrees
        DOWNLEFT(225),         ///< DownLeft, 225 degrees
        LEFT(270),             ///< Left, 270 degrees
        UPLEFT(315),           ///< UpLeft, 315 degrees
        NONE(-1) ;             ///< Not pressed in any direction

        /// \brief the value of the enum
        public final int value ;

        private POVAngle(int value) {
            this.value = value ;
        }

        /// \brief convert to an enum from an integer value
        public static POVAngle fromInt(int id) {
            POVAngle[] As = POVAngle.values() ;
            for(int i = 0 ; i < As.length ; i++) {
                if (As[i].value == id)
                    return As[i] ;
            }

            return NONE ;
        }
    } ;    
}