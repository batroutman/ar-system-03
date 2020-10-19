package ARPipeline;

import Jama.Matrix;

public class EssentialDecomposition {

	protected Matrix R1 = null;
	protected Matrix R2 = null;
	protected Matrix t = null;

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

	public Matrix getT() {
		return t;
	}

	public void setT(Matrix t) {
		this.t = t;
	}

}
