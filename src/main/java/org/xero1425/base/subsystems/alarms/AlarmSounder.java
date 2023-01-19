package org.xero1425.base.subsystems.alarms ;

/// \file

/// \brief This class is a the base class for an AlarmSounder.
/// An AlarmSounder is a class that notifies the drive team that an alarm has
/// expired.  The derived class should implement the signalAlarm() method and signal
/// the drive team.  This is generally done by flashing lights on the OI, 
public abstract class AlarmSounder {

    /// \brief create a new alarm sounder
    /// \param name the name of the sounder
    public AlarmSounder(String name) {
        name_ = name ;
    }

    /// \brief return the name of the AlarmSounder
    /// \returns the name of the AlarmSounder
    public String getName() {
        return name_ ;
    }

    /// \brief signal the drive team that an alarm has expired
    /// This is implemented by a derived class to signal the drive team of an alarm
    public abstract void signalAlarm() ;

    // The name of the sounder
    private String name_ ;
}