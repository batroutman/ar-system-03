package ARPipeline;

import Jama.Matrix;

public class Point3D {

	protected Matrix homogeneousPoint = new Matrix(4, 1);

	public Point3D() {
		this.init();
	}

	public Point3D(double x, double y, double z) {
		this.init();
		this.setX(x);
		this.setY(y);
		this.setZ(z);
	}

	protected void init() {
		this.homogeneousPoint.set(3, 0, 1);
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

	public double getZ() {
		return this.homogeneousPoint.get(2, 0);
	}

	public void setZ(double z) {
		this.homogeneousPoint.set(2, 0, z);
	}

	public Matrix getPoint() {
		return this.homogeneousPoint.getMatrix(0, 2, 0, 0);
	}

	public Matrix getHomogeneousPoint() {
		return this.homogeneousPoint;
	}

}
