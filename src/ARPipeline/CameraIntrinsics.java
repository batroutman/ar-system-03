package ARPipeline;

import Jama.Matrix;

public class CameraIntrinsics {

	public static final float fx = 527.0593f;
	public static final float s = -0.9513f;
	// public static final float s = 0f;
	// public static final float cx = 240.0892f;
	public static final float cx = 240f;
	public static final float fy = 488.0588f;
	// public static final float fy = fx;
	// public static final float cy = 168.4744f;
	public static final float cy = 135f;

	public static Matrix getK() {
		Matrix K = Matrix.identity(3, 3);
		K.set(0, 0, fx);
		K.set(0, 1, s);
		K.set(0, 2, cx);
		K.set(1, 1, fy);
		K.set(1, 2, cy);
		return K;
	}

	public static Matrix getK4x4() {
		Matrix K = Matrix.identity(4, 4);
		K.set(0, 0, fx);
		K.set(0, 1, s);
		K.set(0, 2, cx);
		K.set(1, 1, fy);
		K.set(1, 2, cy);
		return K;
	}

}
