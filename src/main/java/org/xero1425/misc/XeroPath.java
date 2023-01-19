package org.xero1425.misc ;

import java.util.ArrayList ;

/// \file

/// \brief This class represents a single path to be followed by the robot drive base
/// The XeroPath object has a name and a set of X and Y data points for both the left and right sides
/// of the drivebase
public class XeroPath
{
    //
    // The name of the path
    //
    private String name_ ;

    //
    // The set of segment for the left side of the robot
    //
    private ArrayList<ArrayList<XeroPathSegment>> data_ ;

    /// \brief create a new path with the name given
    /// \param name the name of the path
    /// \param data_count the number of data per time point
    public XeroPath(String name, int data_count) throws Exception {
        name_ = name ;
        data_ = new ArrayList<ArrayList<XeroPathSegment>>() ;

        for(int i = 0 ; i < data_count ; i++)
        {
            data_.add(new ArrayList<XeroPathSegment>()) ;
        }
    }

    /// \brief return the path type
    /// \returns the path type
    public int getTrajectorCount() {
        return data_.size() ;
    }

    /// \brief return the name of the path
    /// \returns the name of the path
    public String getName() {
        return name_ ;
    }

    /// \brief returns the number of data points in the path
    /// \returns the numer of data points in the path
    public int getTrajectoryEntryCount() {
        return data_.get(0).size() ;
    }

    /// \brief returns the duration of the path in seconds
    /// \returns the duration of the path in seconds
    public double getDuration() {
        ArrayList<XeroPathSegment> seg = data_.get(0) ;
        return seg.get(seg.size() - 1).getTime() ;
    }

    /// \brief returns a single segment of the path for the requested side of the robot
    /// \param which which wheel to return data for
    /// \param index the index of the segment to return
    /// \returns a single segment of the path for the requested side of the robot
    public XeroPathSegment getSegment(int which, int index) {
        return data_.get(which).get(index) ;
    }

    /// \brief return the segment associated with each wheel
    /// \param index the index of the segment to return
    /// \returns the set of segments for all wheels for the given index
    public XeroPathSegment[] getSegments(int index)
    {
        XeroPathSegment[] ret = new XeroPathSegment[data_.size()] ;
        for(int i = 0 ; i < ret.length ; i++)
        {
            ret[i] = getSegment(i, index) ;
        }

        return ret ;
    }

    /// \brief adds a new path segment to the left adn right sides of the robot
    /// \param which which segment of the path to add to
    /// \param seg the segment value to add to the path
    public void addPathSegment(int which, XeroPathSegment seg) throws Exception
    {
        if (which >= data_.size())
            throw new Exception("invalid wheel index in path") ;

        data_.get(which).add(seg) ;
    }

    /// \brief returns true if the path is valid
    /// \returns true if the path is valid
    public boolean isValid() {
        int size = data_.get(0).size() ;

        for(int i = 1 ; i < data_.size() ; i++)
        {
            if (data_.get(i).size() != size)
                return false ;
        }

        return true ;
    }
}
