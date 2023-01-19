package org.xero1425.base.subsystems.oi;

import java.util.Map;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;

/// \file

/// \brief This class represents an OI Panel that is used by the gunner to control the
/// robot.  A panel consists of a set of OIPanelButton, OIPanelAxisSwitch, and OIPanelAxisScale
/// items.  An OI panel is assocaited with single HID device.
public class OIPanel extends OIDevice
{
    // The next handle value to use when a map() call is made
    int next_handle_ ;

    // The iame from logical item numbers to panel items
    Map<Integer, OIPanelItem> items_ ;

    // The set of LEDs
    List<OILed> leds_ ;

    /// \brief Create a new OI panel assocaited with a HID device
    /// \param sub the subsystem that owns this OIPanel
    /// \param name the name of the OIPanel device
    /// \param index the index of the OIPanel device, used when access the hardware via the WPILib APIs.
    public OIPanel(OISubsystem sub, String name, int index) {
        super(sub, name, index) ;

        leds_ = new ArrayList<OILed>() ;
        items_ = new HashMap<Integer, OIPanelItem>() ;
        next_handle_ = 1 ;
    }

    /// \brief Map a logical button to a physical button and return the handle to the logical button.
    /// Note that multiple logical buttons can be mapped to a single physical button.
    /// \param itemno the physical button number on the targeted HID device.
    /// \param type the type of logical button
    /// \returns the handle to the logical item
    public int mapButton(int itemno, OIPanelButton.ButtonType type) {
        OIPanelButton button = new OIPanelButton(itemno, type) ;
        int handle = next_handle_++ ;
        items_.put(handle, button) ;
        return handle ;
    }

    /// \brief Map a logical knob to a physical axis and return a handle to the logical knob.
    /// \param axisno the physical axis number on the targeted HID device
    /// \param mapping the array to map physical axis value to the logical knob value.
    /// \returns the handle to the logical handle
    public int mapAxisScale(int axisno, Double[] mapping) {
        OIPanelAxisScale scale = new OIPanelAxisScale(axisno, mapping) ;
        int handle = next_handle_++ ;
        items_.put(handle, scale) ;
        return handle ;        
    }

    /// \brief Map a logical knob to a physical axis and return a handle to the logical knob.
    /// \param axisno the physical axis number on the targeted HID device
    /// \param positions the number of values to map the phsical axis to
    /// \returns the handle to the logical handle    
    public int mapAxisSwitch(int axisno, int positions) {
        OIPanelAxisSwitch sw = new OIPanelAxisSwitch(axisno, positions) ;
        int handle = next_handle_++ ;
        items_.put(handle, sw) ;
        return handle ;   
    }

    /// \brief Given the handle to a logical item, return its value
    /// \param handle the handle to a logical item
    /// \returns the value of the logical item if it exists, or -1 if it does not.
    public int getValue(int handle) {
        if (!items_.containsKey(handle))
            return -1 ;
        
        return items_.get(handle).getValue() ;
    }

    public OILed createLED(int io) {
        for(OILed led: leds_) {
            if (led.getIndex() == io)
                return led ;
        }

        OILed led = new OILed(this, io) ;
        leds_.add(led) ;
        return led ;
    }

    /// \brief computes the state of the logical items
    @Override
    public void computeState() throws Exception {

        for(OILed led : leds_)
            led.run() ;
        
        for(OIPanelItem item : items_.values()) {
            if (item.getResourceType() == OIPanelItem.JoystickResourceType.Button)
            {
                boolean v = DriverStation.getStickButton(getIndex(), item.getItemNumber()) ;
                item.setButtonValue(v);
            }
            else
            {
                double v = DriverStation.getStickAxis(getIndex(), item.getItemNumber()) ;
                item.setAxisValue(v) ;
            }
        }
    }

} ;