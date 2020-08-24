package ARPipeline;

import Jama.Matrix;

public class EssentialDecomposition {
	
	protected Matrix R1 = null;
	protected Matrix R2 = null;
	protected Matrix t1 = null;
	protected Matrix t2 = null;

	public EssentialDecomposition() {
		
	}

	public Matrix getR1() {
		return R1;
	}

	public void setR1(Matrix r1) {
		R1 = r1;
	}

	public Matrix getR2() {
		return R2;
	}

	public void setR2(Matrix r2) {
		R2 = r2;
	}

	public Matrix getT1() {
		return t1;
	}

	public void setT1(Matrix t1) {
		this.t1 = t1;
	}

	public Matrix getT2() {
		return t2;
	}

	public void setT2(Matrix t2) {
		this.t2 = t2;
	}
	
}
