package util;

public class Point {

  private double x;
  private double y;
  private double heading;

  public Point(double x, double y) {
    this(x, y, 0);
  }

  public Point(double x, double y, double heading) {
    this.x = x;
    this.y = y;
    this.heading = heading;
  }

  public double distance(Point p) {
    return Math.hypot(this.getX() - p.getX(), this.getY() - p.getY());
  }

  public double getX() {
    return x;
  }

  public void setX(double x) {
    this.x = x;
  }

  public double getY() {
    return y;
  }

  public void setY(double y) {
    this.y = y;
  }

  public double getHeading() {
    return heading;
  }

  public void setHeading(double heading) {
    this.heading = heading;
  }

  public Point copy() {
    return new Point(getX(), getY(), getHeading());
  }

  public void move(double dist) {
    setX(getX() + dist * Math.cos(getHeading()));
    setY(getY() + dist * Math.sin(getHeading()));
  }

  /**
   * Centers the angle around Pi - ranging from 0 to 2Pi
   */
  public void normalize() {
    normalize(Math.PI);
  }

  public static double normalize(double value, double center) {
    while (value > Math.PI * 2 + (center - Math.PI))
      value = (value - Math.PI * 2);
    while (value < 0 + (center - Math.PI))
      value = (value + Math.PI * 2);
    return value;
  }

  public void normalize(double center) {
    while (getHeading() > Math.PI * 2 + (center - Math.PI))
      setHeading(getHeading() - Math.PI * 2);
    while (getHeading() < 0 + (center - Math.PI))
      setHeading(getHeading() + Math.PI * 2);
  }

  @Override
  public boolean equals(Object obj) {
    if (!(obj instanceof Point)) {
      return false;
    }
    Point p = (Point) obj;
    boolean x = getX() - p.getX() < 1e-6;
    boolean y = getY() - p.getY() < 1e-6;
    normalize();
    p.normalize();
    boolean h = getHeading() - p.getHeading() < 1e-6;
    return x && y && h;
  }

  public void substractXY(Point p) {
    this.x -= p.getX();
    this.y -= p.getY();
  }

  public void rotateAroundOrigin(double radians) {
    double newX = x * Math.cos(radians) - y * Math.sin(radians);
    double newY = y * Math.cos(radians) + x * Math.sin(radians);
    setX(newX);
    setY(newY);
  }

  @Override
  public String toString() {
    return "X: " + x + ", Y: " + y + ", H: " + heading;
  }

}
