package util.spline;

import util.Point;

import java.util.ArrayList;
import java.util.List;

public class Path {

	private List<QuinticHermiteSpline> path;

	/**
	 * Doesn't run optimizations
	 * @param path
	 */
	public Path(List<QuinticHermiteSpline> path) {
		this.path = path;
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
