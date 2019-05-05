package util.spline;


import util.Point;

import java.awt.geom.Point2D;

public abstract class Spline {

  public abstract Point2D.Double getPoint2D(double t);

  public abstract double getHeading(double t);

  public final Point getPoint(double t) {
    return new Point(getPoint2D(t).getX(), getPoint2D(t).getY(), getHeading(t));
  }

  public abstract double getCurvature(double t);

  // dk/ds
  public abstract double getDCurvature(double t);

  // ds/dt
  public abstract double getVelocity(double t);

}
