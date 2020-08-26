package ARPipeline;

import java.util.ArrayList;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;
import org.opencv.core.Point;

import Jama.Matrix;

public class MockPointData {

	protected int HEIGHT = 270;
	protected int WIDTH = 480;
	protected long MAX_FRAMES = 480;
	protected int NUM_POINTS = 50;
	protected Matrix K = new Matrix(3, 3);

	// Starting pose parameters
	protected Vector3f initialTranslation = new Vector3f(0f, 0f, 0f);
	protected double initialRotX = 0.0;
	protected double initialRotY = 0.0;
	protected double initialRotZ = 0.0;

	// Amount to update R and t by each frame
	protected Vector3f translationVelocity = new Vector3f(0f, 0f, -0.005f);
	protected double rotX = 0.0;
	protected double rotY = 0.0;
	protected double rotZ = 0.0;

	// List of homogeneous column vectors (4x1) corresponding to world
	// coordinates
	protected ArrayList<Matrix> worldCoordinates = new ArrayList<Matrix>();

	public MockPointData() {

	}

	protected void init() {
		this.initK();
	}

	protected void initK() {
		K.set(0, 0, CameraIntrinsics.fx);
		K.set(0, 1, CameraIntrinsics.s);
		K.set(0, 2, CameraIntrinsics.cx);
		K.set(1, 0, 0.0);
		K.set(1, 1, CameraIntrinsics.fy);
		K.set(1, 2, CameraIntrinsics.cy);
		K.set(2, 0, 0.0);
		K.set(2, 1, 0.0);
		K.set(2, 2, 1.0);
	}

	protected void initWorldCoordinates() {

	}

	// Returns homogeneous 4x4 matrix of JUST rotation parameters
	protected Matrix4f getR(long frameNumber) {
		// Calculate initial R
		Matrix4f initialR = new Matrix4f();
		initialR.setIdentity();
		initialR.rotate((float) this.initialRotX, new Vector3f(1, 0, 0));
		initialR.rotate((float) this.initialRotY, new Vector3f(0, 1, 0));
		initialR.rotate((float) this.initialRotZ, new Vector3f(0, 0, 1));

		// Calculate this frame's rotation parameters
		float gamma = (float) this.rotX * frameNumber;
		float beta = (float) this.rotY * frameNumber;
		float alpha = (float) this.rotZ * frameNumber;

		// Calculate transformation matrix of current frame parameters
		Matrix4f transformation = new Matrix4f();
		transformation.setIdentity();
		transformation.rotate(gamma, new Vector3f(1, 0, 0));
		transformation.rotate(beta, new Vector3f(0, 1, 0));
		transformation.rotate(alpha, new Vector3f(0, 0, 1));

		// Transform inital R and return
		Matrix4f.mul(transformation, initialR, initialR);
		return initialR;
	}

	// Returns homogeneous 4x4 matrix of JUST translation parameters
	protected Matrix4f getT(long frameNumber) {
		// Calculate initial T
		Matrix4f initialT = new Matrix4f();
		initialT.setIdentity();
		initialT.translate(this.initialTranslation);

		// Calculate this frame's translation parameters
		Vector3f translation = new Vector3f(this.translationVelocity.x * frameNumber,
				this.translationVelocity.y * frameNumber, this.translationVelocity.z * frameNumber);

		// Transform initial T and return
		initialT.translate(translation);
		return initialT;
	}

	public ArrayList<Point> getKeypoints(long frameNumber) {
		ArrayList<Point> keypoints = new ArrayList<Point>();

		Matrix R = ARUtils.Matrix4fToMatrix(this.getR(frameNumber));
		Matrix t = ARUtils.Matrix4fToMatrix(this.getT(frameNumber));
		Matrix E = t.times(R);

		Matrix cameraMatrix = this.K.times(E.getMatrix(0, 2, 0, 3));

		for (int i = 0; i < this.worldCoordinates.size(); i++) {
			Matrix point = cameraMatrix.times(this.worldCoordinates.get(i));
			point = point.times(point.get(2, 0));
			Point pt = new Point(point.get(0, 0), point.get(1, 0));
			keypoints.add(pt);
		}

		return keypoints;
	}
}
