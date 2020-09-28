package ARPipeline;

import org.lwjgl.util.vector.Matrix4f;
import org.opencv.core.Mat;

import Jama.Matrix;

public class Pose {

	long timestamp;

	double r00;
	double r01;
	double r02;
	double tx;
	double r10;
	double r11;
	double r12;
	double ty;
	double r20;
	double r21;
	double r22;
	double tz;

	double qw;
	double qx;
	double qy;
	double qz;

	double Cx;
	double Cy;
	double Cz;

	public Pose() {
		timestamp = System.nanoTime();
		r00 = 1;
		r01 = 0;
		r02 = 0;
		r10 = 0;
		r11 = 1;
		r12 = 0;
		r20 = 0;
		r21 = 0;
		r22 = 1;
		tx = 0;
		ty = 0;
		tz = 0;

		qw = 1;
		qx = 0;
		qy = 0;
		qz = 0;

		Cx = 0;
		Cy = 0;
		Cz = 0;
	}

	public void setMatrix(double r00, double r01, double r02, double r10, double r11, double r12, double r20,
			double r21, double r22, double tx, double ty, double tz, long timestamp) {
		this.timestamp = timestamp;
		this.r00 = r00;
		this.r01 = r01;
		this.r02 = r02;
		this.r10 = r10;
		this.r11 = r11;
		this.r12 = r12;
		this.r20 = r20;
		this.r21 = r21;
		this.r22 = r22;
		this.tx = tx;
		this.ty = ty;
		this.tz = tz;
	}

	public void setMatrix(Mat homogeneousMat) {
		this.r00 = homogeneousMat.get(0, 0)[0];
		this.r01 = homogeneousMat.get(0, 1)[0];
		this.r02 = homogeneousMat.get(0, 2)[0];

		this.r10 = homogeneousMat.get(1, 0)[0];
		this.r11 = homogeneousMat.get(1, 1)[0];
		this.r12 = homogeneousMat.get(1, 2)[0];

		this.r20 = homogeneousMat.get(2, 0)[0];
		this.r21 = homogeneousMat.get(2, 1)[0];
		this.r22 = homogeneousMat.get(2, 2)[0];

		this.tx = homogeneousMat.get(0, 3)[0];
		this.ty = homogeneousMat.get(1, 3)[0];
		this.tz = homogeneousMat.get(2, 3)[0];
	}

	public void setMatrix(Matrix homogeneousMat) {
		this.r00 = homogeneousMat.get(0, 0);
		this.r01 = homogeneousMat.get(0, 1);
		this.r02 = homogeneousMat.get(0, 2);

		this.r10 = homogeneousMat.get(1, 0);
		this.r11 = homogeneousMat.get(1, 1);
		this.r12 = homogeneousMat.get(1, 2);

		this.r20 = homogeneousMat.get(2, 0);
		this.r21 = homogeneousMat.get(2, 1);
		this.r22 = homogeneousMat.get(2, 2);

		this.tx = homogeneousMat.get(0, 3);
		this.ty = homogeneousMat.get(1, 3);
		this.tz = homogeneousMat.get(2, 3);
	}

	public void setMatrix(Matrix4f homogeneousMat) {
		this.r00 = homogeneousMat.m00;
		this.r01 = homogeneousMat.m01;
		this.r02 = homogeneousMat.m02;

		this.r10 = homogeneousMat.m10;
		this.r11 = homogeneousMat.m11;
		this.r12 = homogeneousMat.m12;

		this.r20 = homogeneousMat.m20;
		this.r21 = homogeneousMat.m21;
		this.r22 = homogeneousMat.m22;

		this.tx = homogeneousMat.m03;
		this.ty = homogeneousMat.m13;
		this.tz = homogeneousMat.m23;
	}

	public void setOrigin() {
		this.setMatrix(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, System.nanoTime());
	}

	public Matrix4f getHomogeneousMatrix4f() {
		Matrix4f mat = new Matrix4f();
		mat.setIdentity();
		mat.m00 = (float) r00;
		mat.m01 = (float) r01;
		mat.m02 = (float) r02;
		mat.m10 = (float) r10;
		mat.m11 = (float) r11;
		mat.m12 = (float) r12;
		mat.m20 = (float) r20;
		mat.m21 = (float) r21;
		mat.m22 = (float) r22;
		mat.m03 = (float) tx;
		mat.m13 = (float) ty;
		mat.m23 = (float) tz;
		return mat;
	}

	public Matrix getHomogeneousMatrix() {
		Matrix mat = Matrix.identity(4, 4);
		mat.set(0, 0, r00);
		mat.set(0, 1, r01);
		mat.set(0, 2, r02);
		mat.set(0, 3, tx);
		mat.set(1, 0, r10);
		mat.set(1, 1, r11);
		mat.set(1, 2, r12);
		mat.set(1, 3, ty);
		mat.set(2, 0, r20);
		mat.set(2, 1, r21);
		mat.set(2, 2, r22);
		mat.set(2, 3, tz);
		return mat;
	}

	public long getTimestamp() {
		return timestamp;
	}

	public void setTimestamp(long timestamp) {
		this.timestamp = timestamp;
	}

	public double getR00() {
		return r00;
	}

	public void setR00(double r00) {
		this.r00 = r00;
	}

	public double getR01() {
		return r01;
	}

	public void setR01(double r01) {
		this.r01 = r01;
	}

	public double getR02() {
		return r02;
	}

	public void setR02(double r02) {
		this.r02 = r02;
	}

	public double getTx() {
		return tx;
	}

	public void setTx(double tx) {
		this.tx = tx;
	}

	public double getR10() {
		return r10;
	}

	public void setR10(double r10) {
		this.r10 = r10;
	}

	public double getR11() {
		return r11;
	}

	public void setR11(double r11) {
		this.r11 = r11;
	}

	public double getR12() {
		return r12;
	}

	public void setR12(double r12) {
		this.r12 = r12;
	}

	public double getTy() {
		return ty;
	}

	public void setTy(double ty) {
		this.ty = ty;
	}

	public double getR20() {
		return r20;
	}

	public void setR20(double r20) {
		this.r20 = r20;
	}

	public double getR21() {
		return r21;
	}

	public void setR21(double r21) {
		this.r21 = r21;
	}

	public double getR22() {
		return r22;
	}

	public void setR22(double r22) {
		this.r22 = r22;
	}

	public double getTz() {
		return tz;
	}

	public void setTz(double tz) {
		this.tz = tz;
	}

}
