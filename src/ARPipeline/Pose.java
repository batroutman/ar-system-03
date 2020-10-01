package ARPipeline;

import Jama.Matrix;

public class Pose {

	long timestamp;

	double qw;
	double qx;
	double qy;
	double qz;

	double Cx;
	double Cy;
	double Cz;

	public Pose() {
		timestamp = System.nanoTime();

		qw = 1;
		qx = 0;
		qy = 0;
		qz = 0;

		Cx = 0;
		Cy = 0;
		Cz = 0;
	}

	public Matrix getHomogeneousMatrix() {
		Matrix R = this.getRotationMatrix();
		Matrix IC = Matrix.identity(4, 4);
		IC.set(0, 3, -Cx);
		IC.set(1, 3, -Cy);
		IC.set(2, 3, -Cz);

		return R.times(IC);
	}

	public Matrix getRotationMatrix() {
		Matrix R = Matrix.identity(4, 4);
		R.set(0, 0, getR00());
		R.set(0, 1, getR01());
		R.set(0, 2, getR02());
		R.set(1, 0, getR10());
		R.set(1, 1, getR11());
		R.set(1, 2, getR12());
		R.set(2, 0, getR20());
		R.set(2, 1, getR21());
		R.set(2, 2, getR22());
		return R;
	}

	public Matrix getQuaternion() {
		Matrix q = new Matrix(4, 1);
		q.set(0, 0, qw);
		q.set(1, 0, qx);
		q.set(2, 0, qy);
		q.set(3, 0, qz);
		return q;
	}

	public long getTimestamp() {
		return timestamp;
	}

	public void setTimestamp(long timestamp) {
		this.timestamp = timestamp;
	}

	public double getR00() {
		return 1 - 2 * qz * qz - 2 * qy * qy;
	}

	public double getR01() {
		return -2 * qz * qw + 2 * qy * qx;
	}

	public double getR02() {
		return 2 * qy * qw + 2 * qz * qx;
	}

	public double getTx() {
		return -(Cx * getR00() + Cy * getR01() + Cz * getR02());
	}

	public double getR10() {
		return 2 * qx * qy + 2 * qw * qz;
	}

	public double getR11() {
		return 1 - 2 * qz * qz - 2 * qx * qx;
	}

	public double getR12() {
		return 2 * qz * qy - 2 * qx * qw;
	}

	public double getTy() {
		return -(Cx * getR10() + Cy * getR11() + Cz * getR12());
	}

	public double getR20() {
		return 2 * qx * qz - 2 * qw * qy;
	}

	public double getR21() {
		return 2 * qy * qz + 2 * qw * qx;
	}

	public double getR22() {
		return 1 - 2 * qy * qy - 2 * qx * qx;
	}

	public double getTz() {
		return -(Cx * getR20() + Cy * getR21() + Cz * getR22());
	}

	public void normalize() {
		double mag = Math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
		qw /= mag;
		qx /= mag;
		qy /= mag;
		qz /= mag;
	}

	public double getQw() {
		return qw;
	}

	public void setQw(double qw) {
		this.qw = qw;
	}

	public double getQx() {
		return qx;
	}

	public void setQx(double qx) {
		this.qx = qx;
	}

	public double getQy() {
		return qy;
	}

	public void setQy(double qy) {
		this.qy = qy;
	}

	public double getQz() {
		return qz;
	}

	public void setQz(double qz) {
		this.qz = qz;
	}

	public double getCx() {
		return Cx;
	}

	public void setCx(double cx) {
		Cx = cx;
	}

	public double getCy() {
		return Cy;
	}

	public void setCy(double cy) {
		Cy = cy;
	}

	public double getCz() {
		return Cz;
	}

	public void setCz(double cz) {
		Cz = cz;
	}

	public void setT(double tx, double ty, double tz) {
		Matrix RInv = this.getRotationMatrix().getMatrix(0, 0, 2, 2).inverse();
		Matrix T = new Matrix(3, 1);
		T.set(0, 0, tx);
		T.set(1, 0, ty);
		T.set(2, 0, tz);
		Matrix C = RInv.times(T);
		this.Cx = -C.get(0, 0);
		this.Cy = -C.get(1, 0);
		this.Cz = -C.get(2, 0);
	}

}
