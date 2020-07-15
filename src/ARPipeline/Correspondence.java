package ARPipeline;

import org.opencv.core.Mat;

public class Correspondence {
	
	double u1;
	double v1;
	Mat descriptor1 = null;
	
	double u2;
	double v2;
	Mat descriptor2 = null;

	public Correspondence() {
		
	}

	public double getU1() {
		return u1;
	}

	public void setU1(double u1) {
		this.u1 = u1;
	}

	public double getV1() {
		return v1;
	}

	public void setV1(double v1) {
		this.v1 = v1;
	}

	public Mat getDescriptor1() {
		return descriptor1;
	}

	public void setDescriptor1(Mat descriptor1) {
		this.descriptor1 = descriptor1;
	}

	public double getU2() {
		return u2;
	}

	public void setU2(double u2) {
		this.u2 = u2;
	}

	public double getV2() {
		return v2;
	}

	public void setV2(double v2) {
		this.v2 = v2;
	}

	public Mat getDescriptor2() {
		return descriptor2;
	}

	public void setDescriptor2(Mat descriptor2) {
		this.descriptor2 = descriptor2;
	}

}
