package util.spline;

import util.Point;
import util.poofs.InterpolatingDouble;
import util.poofs.InterpolatingTreeMap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Path {

	private List<QuinticHermiteSpline> path;
	//T, dist
	private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> distanceWaypoints;

	/**
	 * Doesn't run optimizations
	 * @param path
	 */
	public Path(List<QuinticHermiteSpline> path) {
		this.path = path;
		generateDistanceWaypoints();
	}

	/**
	 * Runs optimizations
	 * @param points
	 */
	public Path(Point... points) {
		List<QuinticHermiteSpline> splines = new ArrayList<>();
		for(int i = 0; i < points.length-1; i++) {
			splines.add(new QuinticHermiteSpline(points[i], points[i+1]));
		}
		QuinticHermiteSpline.optimizeSpline(splines);
		this.path = splines;
		generateDistanceWaypoints();
	}

	private void generateDistanceWaypoints() {
		distanceWaypoints = new InterpolatingTreeMap<>();
		double d = 0;
		final double dT = 1e-6;
		for(double t = 0; t < size(); t += dT) {
			distanceWaypoints.put(new InterpolatingDouble(t), new InterpolatingDouble(d));
			d += getPoint(t).distance(getPoint(t + dT));
		}
	}

	public double getDistance(double t) {
		return distanceWaypoints.getInterpolated(new InterpolatingDouble(t)).value;
	}

	public double getDistanceBetween(double start, double end) {
		return getDistance(end) - getDistance(start);
	}

	public Point getPoint(double i) {
		if(i < 0) {
			return path.get(0).getPoint(0);
		}
		if(i >= path.size()) {
			return path.get(path.size()-1).getPoint(1);
		}
		return path.get((int) i).getPoint(i % 1);
	}

	public double  getCurvature(double t) {
		if(t < 0) {
			return path.get(0).getCurvature(0);
		}
		if(t >= path.size()) {
			return path.get(path.size()-1).getCurvature(1);
		}
		return path.get((int) t).getCurvature(t % 1);
	}

	public double getDCurvature(double t) {
		if(t < 0) {
			return path.get(0).getDCurvature(0);
		}
		if(t >= path.size()) {
			return path.get(path.size()-1).getDCurvature(1);
		}
		return path.get((int) t).getDCurvature(t % 1);
	}

	public double getHeading(double t) {
		if(t < 0) {
			return path.get(0).getHeading(0);
		}
		if(t >= path.size()) {
			return path.get(path.size()-1).getHeading(1);
		}
		return path.get((int) t).getHeading(t % 1);
	}

	public List<QuinticHermiteSpline> getPath() {
		return path;
	}

	public Spline get(int i) {
		if(i >= path.size()) {
			return path.get(path.size()-1);
		}
		if (i < 0) {
			return path.get(0);
		}
		return path.get(i);
	}

	public int size() {
		return path.size();
	}

}
