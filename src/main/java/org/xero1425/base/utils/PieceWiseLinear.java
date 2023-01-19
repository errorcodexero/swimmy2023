package org.xero1425.base.utils;

/// \file

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.SettingsValue;

import edu.wpi.first.math.geometry.Translation2d;

/// \brief this class represents an XY function that consists of a series of line segments
public class PieceWiseLinear {
    private List<Translation2d> points_;

    /// \brief This constructor creates the PieceWiseLinear object from the settings file.
    /// The data will be stored in the file as "basename:#:x" and "basename:#:y" where
    /// the # value starts at 1 is incremented for each data point that makes up the
    /// piecewise linear value.  The points are sorted on the X value so they do not have
    /// to be in order.
    ///
    /// \param settings settings parser used to extract values from settings file
    /// \param basename the base name of the data
    public PieceWiseLinear(ISettingsSupplier settings, String basename) throws Exception {
        List<Translation2d> points = readFromSettings(settings, basename);
        init(points) ;
    }

    /// \brief This constructor creates the PieceWiseLinear object from a set of points given.
    /// The points are sorted on the X value so they do not have to be in order.
    /// \param points the set of points provided
    public PieceWiseLinear(List<Translation2d> points) throws Exception {
        init(points) ;
    }

    /// \brief This method returns the number of points that make up the piece wise linear function
    /// \returns the number of points that make up the piece wise linear function
    public int size() {
        return points_.size();
    }

    /// \brief This method returns a point given its index.
    /// Note, this function returns the point after sorting, so the values returned will not
    /// match the values provided if the original values were not sorted on the X value.
    /// \returns a point given its index
    public Translation2d get(int i) {
        return points_.get(i);
    }

    /// \brief This method returns a Y value given an X value.
    /// If the X value supplied is before the first X value of the piece wise linear function, the
    /// Y value of the first point is returned.  If the X value supplied is after the X value of the
    /// last X value of the piece wise linear function, the Y value of the last point is returned.  If
    /// the X value supplied is within the points of the piece wise linear function, the segment containing
    /// the X value is found an a Y value is retured based on linear interpolation between the start and end
    /// of the containing segment.
    /// \returns the Y value given the X value.
    public double getValue(double x) {
        double ret;

        if (x < points_.get(0).getX()) {
            ret = points_.get(0).getY();
        } else if (x > points_.get(points_.size() - 1).getX()) {
            ret = points_.get(points_.size() - 1).getY();
        } else {
            int which_x = findWhich(x);

            Translation2d low = points_.get(which_x);
            Translation2d high = points_.get(which_x + 1);

            double m = ((high.getY() - low.getY()) / (high.getX() - low.getX()));
            double b = high.getY() - (high.getX() * m);

            ret = (m * x) + b;

        }

        return ret;
    }

    private void init(List<Translation2d> points) throws Exception {
        points_ = new ArrayList<Translation2d>();

        if (points.size() == 0)
            throw new Exception("cannot have empty points list as input");

        for (int i = 0; i < points.size(); i++)
            points_.add(points.get(i));

        Collections.sort(points_, new Comparator<Translation2d>() {
            public int compare(Translation2d p1, Translation2d p2) {
                if (p1.getX() < p2.getX())
                    return -1;

                if (p1.getX() > p2.getX())
                    return 1;

                if (p1.getY() < p2.getY())
                    return -1;

                if (p1.getY() > p2.getY())
                    return 1;

                return 0;
            }
        });
    }

    private int findWhich(double x) {
        for (int i = 0; i < points_.size(); i++) {
            if (x >= points_.get(i).getX() && x < points_.get(i + 1).getX()) {
                return i;
            }
        }

        return -1;
    }

    private List<Translation2d> readFromSettings(ISettingsSupplier settings, String base)
            throws BadParameterTypeException {
        int index = 1 ;
        List<Translation2d> points = new ArrayList<Translation2d>() ;

        while (true) {
            String xname = base + ":" + Integer.toString(index) + ":x" ;
            String yname = base + ":" + Integer.toString(index) + ":y" ;

            SettingsValue xvalue = settings.getOrNull(xname) ;
            SettingsValue yvalue = settings.getOrNull(yname) ;

            if (xvalue == null || yvalue == null)
                break ;

            Translation2d pt = new Translation2d(xvalue.getDouble(), yvalue.getDouble()) ;
            points.add(pt) ;

            index++ ;
        }

        return points ;
    }
}