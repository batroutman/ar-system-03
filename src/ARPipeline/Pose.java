package ARPipeline;

public class Pose {
	
	long timestamp;

	float r00; float r01; float r02; float tx;
	float r10; float r11; float r12; float ty;
	float r20; float r21; float r22; float tz;
	
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
	
	public void setMatrix(float r00, float r01, float r02, float r10, float r11, float r12, float r20, float r21, float r22, float tx, float ty, float tz, long timestamp) {
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
	
	public void setOrigin() {
		this.setMatrix(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, System.nanoTime());
	}
	
	public long getTimestamp() {
		return timestamp;
	}

	public void setTimestamp(long timestamp) {
		this.timestamp = timestamp;
	}

	public float getR00() {
		return r00;
	}

	public void setR00(float r00) {
		this.r00 = r00;
	}

	public float getR01() {
		return r01;
	}

	public void setR01(float r01) {
		this.r01 = r01;
	}

	public float getR02() {
		return r02;
	}

	public void setR02(float r02) {
		this.r02 = r02;
	}

	public float getTx() {
		return tx;
	}

	public void setTx(float tx) {
		this.tx = tx;
	}

	public float getR10() {
		return r10;
	}

	public void setR10(float r10) {
		this.r10 = r10;
	}

	public float getR11() {
		return r11;
	}

	public void setR11(float r11) {
		this.r11 = r11;
	}

	public float getR12() {
		return r12;
	}

	public void setR12(float r12) {
		this.r12 = r12;
	}

	public float getTy() {
		return ty;
	}

	public void setTy(float ty) {
		this.ty = ty;
	}

	public float getR20() {
		return r20;
	}

	public void setR20(float r20) {
		this.r20 = r20;
	}

	public float getR21() {
		return r21;
	}

	public void setR21(float r21) {
		this.r21 = r21;
	}

	public float getR22() {
		return r22;
	}

	public void setR22(float r22) {
		this.r22 = r22;
	}

	public float getTz() {
		return tz;
	}

	public void setTz(float tz) {
		this.tz = tz;
	}
	
}
