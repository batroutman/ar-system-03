package ARPipeline;

import org.opencv.core.Mat;

public class Correspondence2D2D {

	protected Double u1 = null;
	protected Double v1 = null;
	protected Double u2 = null;
	protected Double v2 = null;
	protected Mat descriptor1 = null;
	protected Mat descriptor2 = null;
	protected Double du = null;
	protected Double dv = null;

	public Correspondence2D2D() {

	}

	public Correspondence2D2D(Double u1, Double v1, Double u2, Double v2) {
		this.set(u1, v1, u2, v2);
	}

	public void set(Double u1, Double v1, Double u2, Double v2) {
		this.setU1(u1);
		this.setV1(v1);
		this.setU2(u2);
		this.setV2(v2);
	}

	public Double getU1() {
		return u1;
	}

	public void setU1(Double u1) {
		this.u1 = u1;
	}

	public Double getV1() {
		return v1;
	}

	public void setV1(Double v1) {
		this.v1 = v1;
	}

	public Double getU2() {
		return u2;
	}

	public void setU2(Double u2) {
		this.u2 = u2;
	}

	public Double getV2() {
		return v2;
	}

	public void setV2(Double v2) {
		this.v2 = v2;
	}

	public Mat getDescriptor1() {
		return descriptor1;
	}

	public void setDescriptor1(Mat descriptor1) {
		this.descriptor1 = descriptor1;
	}

	public Mat getDescriptor2() {
		return descriptor2;
	}

	public void setDescriptor2(Mat descriptor2) {
		this.descriptor2 = descriptor2;
	}

	public Double getDu() {
		return du;
	}

	public void setDu(Double du) {
		this.du = du;
	}

	public Double getDv() {
		return dv;
	}

	public void setDv(Double dv) {
		this.dv = dv;
	}

}
