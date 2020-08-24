package ARPipeline;

import Jama.Matrix;

public class Rt {
	
	protected Matrix R = null;
	protected Matrix t = null;

	public Rt() {
		
	}

	public Matrix getR() {
		return R;
	}

	public void setR(Matrix r) {
		R = r;
	}

	public Matrix getT() {
		return t;
	}

	public void setT(Matrix t) {
		this.t = t;
	}
	
}
