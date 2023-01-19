package org.xero1425.base.misc;

import org.xero1425.base.XeroRobot;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class XeroTimer {
    private static int LoggerID = -1 ;
    private static final String LoggerIDName = "XeroTimer" ;
    private XeroRobot robot_ ;
    private boolean running_ ;
    private double duration_ ;
    private double endtime_ ;
    private double start_ ;
    private String name_ ;

    public XeroTimer(XeroRobot robot, String name, double duration) {
        robot_ = robot ;
        name_ = name ;
        duration_ = duration ;
        running_ = false ;
        endtime_ = 0.0 ;        

        if (LoggerID == -1) {
            LoggerID = robot_.getMessageLogger().registerSubsystem(LoggerIDName) ;
        }
    }

    public double getDuration() {
        return duration_ ;
    }

    public void setDuration(double dur) {
        if (running_) {
            MessageLogger logger = robot_.getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("Timer ").add(name_).add(" had duration updated while running - change ignored").endMessage();
        }

        duration_ = dur ;
    }

    public double elapsed() {
        return robot_.getTime() - start_ ;
    }

    public void start() {
        if (running_) {
            MessageLogger logger = robot_.getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("Timer ").add(name_).add(" was started while running").endMessage();
        }

        running_ = true ;
        start_ = robot_.getTime() ;
        endtime_ = start_ + duration_ ;
    }

    public boolean isRunning() {
        return running_ ;
    }

    public boolean isExpired() {
        boolean ret = false ;

        if (running_ && robot_.getTime() > endtime_) {
            running_ = false ;
            ret = true ;
        }

        return ret ;
    }
}