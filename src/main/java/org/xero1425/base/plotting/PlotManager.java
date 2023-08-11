package org.xero1425.base.plotting ;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry ;
import java.util.Map ;

import org.xero1425.base.XeroRobot;

import java.util.ArrayList;
import java.util.HashMap ;
import java.util.List;

/// \file

/// \brief This class manages "plots". 
///
/// Plots are data sets that are stored to the network table.  The data stored there
/// contains a set of points, per robot loop, for each robot loop that is processed while
/// the plot is ednabled.  A plot is defined by a name and a set of named columns.  Each named
/// column has a value for each robot loop.
/// This data can be processed by the xerotune toon located here <a href="https://www.mewserver.org/xeroprogs/" here </a>
public class PlotManager extends PlotManagerBase
{
    private class DelayedPlotEndRequests
    {
        public int id_ ;
        public double endtime_ ;

        public DelayedPlotEndRequests(int id, double endtime) {
            id_ = id ;
            endtime_ = endtime ;
        }
    }

    private class PlotInfo
    {
        public String name_ ;                   // The human readable name for the plot
        public PlotDataSource datasrc_ ;        // THe data source for the plot
        public int id_ ;                        // The ID of the plot
        public int index_ ;                     // The current data index in the network tables

        PlotInfo(String name, int id, PlotDataSource src) {
            name_ = name ;
            id_ = id ;
            index_ = 0 ;
            datasrc_ = src ;
        }
    } ;

    static private String CompleteEntry = "complete" ;
    static private String PointsEntry = "points" ;
    static private String ColumnsEntry = "columns" ;
    static private String DataEntry = "data" ;
    static private String VersionEntry = "version" ;
    
    private int next_plot_id_ ;
    private String plot_table_ ;
    private Map<Integer, PlotInfo> plots_ ;
    private List<DelayedPlotEndRequests> delayed_ ;

    /// \brief create a new plot manager
    /// \param key the name of the key in the network table to hold plot data
    public PlotManager(XeroRobot robot, String key)
    {
        super(robot) ;

        plots_ = new HashMap<Integer, PlotInfo>() ;
        delayed_ = new ArrayList<DelayedPlotEndRequests>() ;
        next_plot_id_ = 0 ;
        plot_table_ = key ;
    }

    public void run() {
        ArrayList<DelayedPlotEndRequests> remove = new ArrayList<DelayedPlotEndRequests>() ;

        //
        // Extract data for all active plots and send to network tables
        // 
        for(PlotInfo info : plots_.values()) {
            addPlotData(info);
        }

        //
        // Check for any plots with a delayed end time and see if it is time to shut them down
        //
        double now = getRobot().getTime() ;
        for(DelayedPlotEndRequests req : delayed_) {
            if (now > req.endtime_) {
                endPlot(req.id_);
                remove.add(req);
            }
        }

        for(DelayedPlotEndRequests req : remove) {
            delayed_.remove(req);
        }
    }

    public int initPlot(String name, PlotDataSource src)
    {
        if (!isPlotEnabled(name))
            return -1 ;

        for(int key : plots_.keySet())
        {
            if (plots_.get(key).name_ == name)
                return key ;
        }

        PlotInfo info = new PlotInfo(name, next_plot_id_++, src) ;
        plots_.put(info.id_, info) ;
        return info.id_ ;
    }

    public void startPlot(int id)
    {
        PlotInfo info = plots_.get(id) ;
        if (info == null || !isPlotEnabled(info.name_))
            return ;

        info.index_ = 0 ;

        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;
        NetworkTable table = inst.getTable(getKeyForPlot(id)) ;
        NetworkTableEntry entry ;
        
        entry = table.getEntry(ColumnsEntry) ;
        entry.setStringArray(info.datasrc_.columns());

        entry = table.getEntry(PointsEntry) ;
        entry.setNumber(0) ;

        entry = table.getEntry(VersionEntry) ;
        entry.setNumber(3) ;

        entry = table.getEntry(CompleteEntry) ;
        entry.setBoolean(false) ;

        inst.flush() ;
    }

    private void addPlotData(PlotInfo info)
    {
        Double data[] = info.datasrc_.values() ;
        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;
        NetworkTable table = inst.getTable(getKeyForPlot(info.id_)) ;
        NetworkTableEntry entry = table.getEntry(DataEntry + "/" + Integer.toString(info.index_)) ;
        entry.setNumberArray(data) ;
        entry = table.getEntry(PointsEntry) ;
        info.index_++ ;
        entry.setNumber(info.index_) ;
    }

    public void endPlot(int id)
    {
        PlotInfo info = plots_.get(id) ;
        if (info == null || !isPlotEnabled(info.name_))
            return ;
            
        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;
        NetworkTable table = inst.getTable(getKeyForPlot(id)) ;
        NetworkTableEntry entry = table.getEntry(CompleteEntry) ;
        entry.setBoolean(true) ;

        inst.flush() ;
    }

    public void endPlot(int id, double delay)
    {
        DelayedPlotEndRequests req = new DelayedPlotEndRequests(id, getRobot().getTime() + delay) ;
        delayed_.add(req) ;
    }

    private String getKeyForPlot(int id)
    {
        if (!plots_.containsKey(id))
            return null ;

        PlotInfo info = plots_.get(id) ;
        return plot_table_ + "/" + info.name_ ;
    }

} ;