package ARPipeline;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class Pose {
	
	long timestamp;

	double r00; double r01; double r02; double tx;
	double r10; double r11; double r12; double ty;
	double r20; double r21; double r22; double tz;
	
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
	}
	
	public void setMatrix(double r00, double r01, double r02, double r10, double r11, double r12, double r20, double r21, double r22, double tx, double ty, double tz, long timestamp) {
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
		
		this.tx = homogeneousMat.get(0,  3)[0];
		this.ty = homogeneousMat.get(1,  3)[0];
		this.tz = homogeneousMat.get(2,  3)[0];
	}
	
	public void setOrigin() {
		this.setMatrix(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, System.nanoTime());
	}
	
	public Mat getHomogeneous() {
		Mat mat = Mat.zeros(4, 4, CvType.CV_32F);
		mat.put(0,  0, this.r00);
		mat.put(0,  1, this.r01);
		mat.put(0,  2, this.r02);
		mat.put(0,  3, this.tx);
		mat.put(1,  0, this.r10);
		mat.put(1,  1, this.r11);
		mat.put(1,  2, this.r12);
		mat.put(1,  3, this.ty);
		mat.put(2,  0, this.r20);
		mat.put(2,  1, this.r21);
		mat.put(2,  2, this.r22);
		mat.put(2,  3, this.tz);
		mat.put(3,  3, 1.0f);
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
