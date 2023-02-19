package org.xero1425.base;

public interface IVisionAlignmentData {

    /// \brief the mode for the camera
    public enum CamMode
    {
        VisionProcessing,           ///< Processing vision targets on the field
        DriverCamera,               ///< Relaying the image to the driver on the driver station
        Invalid                     ///< Invalid camera mode
    } ;

    /// \brief the mode for the LED
    public enum LedMode
    {
        UseLED,                     ///< Use the LED per the pipeline
        ForceOff,                   ///< Force the LED to the off state
        ForceBlink,                 ///< Force the LED to a blinking state
        ForceOn,                    ///< Force the LED to the on state
        Invalid                     ///< Invalid state
    } ;
    
    boolean isTargetDetected();
    double getTX();
    double getTY();
    void setCamMode(CamMode mode) ;
    void setLedMode(LedMode mode) ;
}
