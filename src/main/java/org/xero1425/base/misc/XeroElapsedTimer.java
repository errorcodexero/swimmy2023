package org.xero1425.base.misc;

import org.xero1425.base.XeroRobot;

/// \file

/// \brief Count-up timer. Measures elapsed time since object creation or last reset.
public class XeroElapsedTimer {
    private XeroRobot robot_ ;
    private double start_ ;

    public XeroElapsedTimer(XeroRobot robot) {
        robot_ = robot ;
        reset();
    }

    public void reset() {
        start_ = robot_.getTime() ;
    }

    public double elapsed() {
        return robot_.getTime() - start_ ;
    }

}