package org.xero1425.base.subsystems.alarms;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.subsystems.Subsystem;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.* ;

/// \file

/// \brief this subsystem provides a mechanism for robot code to set alarms.
/// The alarms are set as a function of the match time, starting from the beginning
/// of the match.  Alarms are presented to the drive team in terms of AlarmSounders. 
/// These can be flashing lights on the OI, vibrating the gamepad, etc.
public class AlarmSubsystem extends Subsystem {
    
    // List of alarms to monitor
    private List<AlarmEntry> entries_ ;

    /// \brief create the AlarmSubsystem.
    /// \param parent the parent subsystem
    public AlarmSubsystem(Subsystem parent) {
        super(parent, "alarms") ;

        entries_ = new ArrayList<AlarmEntry>() ;
    }

    @Override
    public String getStatus() {
        String st = "" ;

        if (entries_.size() == 0) {
            st += "No Alarms Set" ;
        }
        else {
            for(AlarmEntry entry : entries_) {
                st += "Alarm @ " + entry.getTime() + "<br>" ;
            }
        }
        return st ;
    }

    /// \brief run the AlarmSubsystem.
    /// Running this subsystem checks each active alarm and determines if it
    /// has expired.  If the alarm is expired the associated AlarmSounder is used
    /// to communicate this to the drive team.
    @Override
    public void run() {
        if (DriverStation.isFMSAttached()) {
            double remaining = DriverStation.getMatchTime() ;

            while (!entries_.isEmpty())
            {
                if (remaining < entries_.get(0).getTime())
                {
                    entries_.get(0).getSounder().signalAlarm();
                    entries_.remove(0) ;
                }
                else 
                {
                    break ;
                }
            }
        }
    }

    /// \brief add a new alarm
    /// \param time the match time remaining for the alarm (30 would mean 30 seconds remaining in the match)
    /// \param sounder the object used to notify the drive team that the alarm has expired
    public void addEntry(double time, AlarmSounder sounder) {
        AlarmEntry entry = new AlarmEntry(time, sounder) ;
        entries_.add(entry) ;

        Collections.sort(entries_, new SortByTime()) ;
    }

    private class AlarmEntry {
        public AlarmEntry(double time, AlarmSounder sounder) {
            time_ = time ;
            sounder_ = sounder ;
        }

        public double getTime() {
            return time_ ;
        }

        public AlarmSounder getSounder() {
            return sounder_ ;
        }

        private double time_ ;
        private AlarmSounder sounder_ ;
    } ;

    private class SortByTime implements Comparator<AlarmEntry> {
        public int compare(AlarmEntry a, AlarmEntry b) {
            double diff  = a.getTime() - b.getTime() ;
            if (diff < 0.0)
                return -1 ;
            else if (diff > 0.0)
                return 1 ;

            return 0 ;
        }
    }

}