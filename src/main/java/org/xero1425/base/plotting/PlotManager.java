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
        public double when_ ;

        public DelayedPlotEndRequests(int id, double when) {
            id_ = id ;
            when_ = when ;
        }
    }

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

    public int initPlot(String name)
    {
        if (!isPlotEnabled(name))
            return -1 ;

        for(int key : plots_.keySet())
        {
            if (plots_.get(key).name_ == name)
                return key ;
        }

        PlotInfo info = new PlotInfo(name, next_plot_id_++) ;
        plots_.put(info.index_, info) ;

        return info.index_ ;
    }

    public void startPlot(int id, PlotDataSource src)
    {
        PlotInfo info = plots_.get(id) ;
        if (info == null || !isPlotEnabled(info.name_))
            return ;

        info.datasrc_ = src ;
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

    public void addPlotData(int id)
    {
        PlotInfo info = plots_.get(id) ;
        if (info == null || !isPlotEnabled(info.name_))
            return ;
            
        Double data[] = info.datasrc_.values() ;
        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;
        NetworkTable table = inst.getTable(getKeyForPlot(id)) ;
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

    private class PlotInfo
    {
        PlotInfo(String name, int index) {
            name_ = name ;
            index_ = index ;
        }
        public String name_ ;
        public PlotDataSource datasrc_ ;
        public int index_ ;
    } ;
} ;