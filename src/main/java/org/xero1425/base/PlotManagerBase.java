package org.xero1425.base;

import java.util.ArrayList;
import java.util.List;

public abstract class PlotManagerBase {

    private List<String> enabled_plots_ ;
    private XeroRobot robot_ ;
    private boolean enabled_ ;

    public PlotManagerBase(XeroRobot robot) {
        enabled_plots_ = new ArrayList<String>() ;
        robot_ = robot ;
    }

    public abstract int initPlot(String name) ;
    public abstract void startPlot(int id, String[] cols) ;
    public abstract void addPlotData(int id, Double[] data) ;
    public abstract void endPlot(int id) ;

    public void enable(boolean value) {
        enabled_ = value ;
    }

    public void enable(String name) {
        enabled_plots_.add(name);
    }

    protected boolean isPlotEnabled(String name) {
        boolean ret = false ;

        if (!robot_.shutdownDebug()) {
            if (enabled_ == true && enabled_plots_.size() > 0) {
                if (enabled_plots_.contains(name))
                    ret = true ;
            }
            else if (enabled_ == true) {
                ret = true ;
            }
        }

        return ret;
    }
}
