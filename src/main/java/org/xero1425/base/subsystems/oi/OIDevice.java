package org.xero1425.base.subsystems.oi;

import org.xero1425.base.LoopType;
import org.xero1425.base.actions.InvalidActionRequest;

import edu.wpi.first.wpilibj.GenericHID;

/// \file

/// \brief This class represents a basic HID device attached to the driver station.  This
/// could be a gamepad for driving the robot, or an OI device for the gunner.
public abstract class OIDevice
{
    // The name of the device
    private String name_ ;

    // If true, this device is enabled
    private boolean enabled_ ;

    // The OI subsystem that owns this device
    private OISubsystem sub_ ;

    // The index for this HIDDevice in the WPILib API called.
    private int index_ ;

    // The WPILib HID device for this device
    private GenericHID hid_device_ ;

    /// \brief Create a new HID device
    /// \param sub the subsystem that owns this HID device
    /// \param name the name of this HID device
    /// \param index the index used to reference this device in the WPILib APIs.
    public OIDevice(OISubsystem sub, String name, int index) {
        sub_ = sub ;
        index_ = index ;
        enabled_ = true ;
        name_ = name ;

        hid_device_ = new GenericHID(index) ;
    }

    public OIDevice(OISubsystem sub, String name) {
        index_ = -1 ;
        sub_ = sub ;
        enabled_ = true ;
        name_ = name ;
        hid_device_ = null ;
    }

    public void initAutoModes() {

    }

    /// \brief Return the the name of the device
    /// \returns the name of the device
    public String getName() {
        return name_ ;
    }

    /// \brief Return the subsystem that owns this device
    /// \returns the subsystem that owns the device
    public OISubsystem getSubsystem() {
        return sub_ ;
    }

    /// \brief Return the index for the device, used in WPILib calls
    /// \returns the index for the device, used in WPILib calls
    public int getIndex() throws Exception {
        if (index_ == -1) {
            throw new Exception("calling getIndex() when the OIDevice is not HID based") ;
        }
        return index_ ;
    }

    /// \brief Called when the mode is initializating
    /// \param ltype the mode being initialized
    public void init(LoopType ltype) {
    }

    /// \brief Called each robot loop to generate actions
    /// \param seq the sequence where generated actions are added
    /// \throws InvalidActionRequest when an invalid action reqested
    public void generateActions() throws InvalidActionRequest {
    }

    /// \brief Create the set of static actions that will be reused each robot loop
    public void createStaticActions() throws Exception {
    }

    /// \brief Compute the current state of the HID device.
    public abstract void computeState() throws Exception ;

    /// \brief Return the state of the auto mode selector, if one exists.  Otherwise return -1 if 
    /// this device does not contain an auto mode selector
    public int getAutoModeSelector() {
        return Integer.MAX_VALUE ;
    }

    /// \brief enable this HID device
    public void enable() {
        enabled_ = true ;
    }

    /// \brief disable this HID device
    public void disable() {
        enabled_ = false ;
    }

    /// \brief returns true if this device is enabled
    /// \returns true if this device is enabled
    public boolean isEnabled() {
        return enabled_ ;
    }

    public void setOutput(int output, boolean value) {
        hid_device_.setOutput(output, value) ;
    }
}