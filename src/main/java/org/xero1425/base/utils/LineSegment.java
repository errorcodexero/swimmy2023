package org.xero1425.base.utils;

/// \file

import edu.wpi.first.math.geometry.Translation2d;

/// \brief This class represents a line segment with a start end end point
public class LineSegment {
    private Translation2d p1_ ;
    private Translation2d p2_ ;

    /// \brief Create a line segment given two points
    /// \param p1 the starting point for the segment
    /// \param p2 the ending point for the segment
    public LineSegment(Translation2d p1, Translation2d p2) {
        p1_ = new Translation2d(p1.getX(), p1.getY()) ;
        p2_ = new Translation2d(p2.getX(), p2.getY());
    }

    /// \brief Create a line segment given two points
    /// \param x1 the X value for the starting point for the segment
    /// \param y1 the Y value for the starting point for the segment
    /// \param x2 the X value for the ending point for the segment
    /// \param y2 the Y value for the ending point for the segment            
    public LineSegment(double x1, double y1, double x2, double y2) {
        p1_ = new Translation2d(x1, y1) ;
        p2_ = new Translation2d(x2, y2) ;
    }

    /// \brief returns the length of the segment
    /// \returns the length of the segment
    public double length() {
        return p1_.getDistance(p2_) ;
    }

    /// \brief returns the dot product of the line segment and a vector from the first point of the line segment and the point provided
    /// \param pt the point defining the second vector
    /// \returns the doc product of the linesegment and a point, returns -1 if the dot product is not defined
    public double dotProd(Translation2d pt)
    {
        Translation2d pt_p1 = pt.minus(p1_) ;
        Translation2d p2_p1 = p2_.minus(p1_) ;
        return pt_p1.getX() * p2_p1.getX() + pt_p1.getY() * p2_p1.getY() ;
    }

    /// \brief returns the point on the line segment closest to the point given
    /// \param pt the point we are supplied
    /// \returns the point on the line segment closest to the point supplied
    public Translation2d closest(Translation2d pt) {
        Translation2d ret = null ;
        Translation2d pt_p1 = pt.minus(p1_) ;
        Translation2d p2_p1 = p2_.minus(p1_) ;
        double dot = pt_p1.getX() * p2_p1.getX() + pt_p1.getY() * p2_p1.getY() ;
        double lensq = p2_p1.getX() * p2_p1.getX() + p2_p1.getY() * p2_p1.getY() ;
        double param = -1 ;

        if (lensq != 0.0)
            param = dot / lensq ;

        if (param < 0.0) {
            //
            // The point is closeest to point P1
            //
            ret = p1_ ;
        }
        else if (param >= 0.0 && param <= 1.0)
        {
            double x = p1_.getX() + param * p2_p1.getX() ;
            double y = p1_.getY() + param * p2_p1.getY() ;

            ret = new Translation2d(x, y) ;
        }
        else
        {
            //
            // The point is closest to point P2
            //
            ret = p2_ ;
        }

        return ret ;
    }
}
