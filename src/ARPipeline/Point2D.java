package ARPipeline;

import Jama.Matrix;

public class Point2D {

	protected Matrix homogeneousPoint = new Matrix(3, 1);

	public Point2D() {
		this.init();
	}

	public Point2D(double x, double y) {
		this.init();
		this.setX(x);
		this.setY(y);
	}

	protected void init() {
		this.homogeneousPoint.set(2, 0, 1);
	}

	public double getX() {
		return this.homogeneousPoint.get(0, 0);
	}

	public void setX(double x) {
		this.homogeneousPoint.set(0, 0, x);
	}

	public double getY() {
		return this.homogeneousPoint.get(1, 0);
	}

	public void setY(double y) {
		this.homogeneousPoint.set(1, 0, y);
	}

	public Matrix getPoint() {
		return this.homogeneousPoint.getMatrix(0, 1, 0, 0);
	}

	public Matrix getHomogeneousPoint() {
		return this.homogeneousPoint;
	}

}
