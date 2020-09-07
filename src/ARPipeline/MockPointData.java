package ARPipeline;

import java.util.ArrayList;
import java.util.Random;

import org.lwjgl.util.vector.Vector3f;
import org.opencv.core.Point;

import Jama.Matrix;

public class MockPointData {

	protected int HEIGHT = 270;
	protected int WIDTH = 480;
	protected long MAX_FRAMES = 100;
	protected int NUM_POINTS = 30;
	protected int START_FRAME = 0;
	protected int SEED = 1;
	protected Matrix K = new Matrix(3, 3);

	// Starting pose parameters
	// NOTE: translations should be negative (-C)
	protected Vector3f initialTranslation = new Vector3f(0f, 0f, 500f);
	protected double initialRotX = 0.0;
	protected double initialRotY = 0.0;
	protected double initialRotZ = 0.0;

	// Amount to update R and t by each frame
	// NOTE: translations should be negative (-C)
	protected Vector3f translationVelocity = new Vector3f(3f, 1f, -10f);
	// protected Vector3f translationVelocity = new Vector3f(5f, 3f, -10f);
	protected double rotX = 0.002;
	protected double rotY = -0.005;
	protected double rotZ = -0.000;

	// List of homogeneous column vectors (4x1) corresponding to world
	// coordinates
	protected ArrayList<Matrix> worldCoordinates = new ArrayList<Matrix>();

	public MockPointData() {
		this.init();
	}

	protected void init() {
		this.initK();
		this.initWorldCoordinates();
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
		String output = "";
		double Z_SPAWN_MIN = 1000;
		double Z_SPAWN_MAX = 1500;
		double Y_SPAWN_MIN = -150;
		double Y_SPAWN_MAX = 150;
		double X_SPAWN_MIN = -200;
		double X_SPAWN_MAX = 200;

		double Z_RANGE = Z_SPAWN_MAX - Z_SPAWN_MIN;
		double Y_RANGE = Y_SPAWN_MAX - Y_SPAWN_MIN;
		double X_RANGE = X_SPAWN_MAX - X_SPAWN_MIN;

		Random random = new Random(this.SEED);
		for (int i = 0; i < this.NUM_POINTS; i++) {
			Matrix point = new Matrix(4, 1);
			point.set(0, 0, random.nextDouble() * X_RANGE + X_SPAWN_MIN);
			point.set(1, 0, random.nextDouble() * Y_RANGE + Y_SPAWN_MIN);
			point.set(2, 0, random.nextDouble() * Z_RANGE + Z_SPAWN_MIN);
			point.set(3, 0, 1);
			this.worldCoordinates.add(point);
			// output += point.get(0, 0) + ", " + point.get(1, 0) + ", " +
			// point.get(2, 0) + "\n";
		}
		// System.out.println(output);

	}

	// Returns homogeneous 4x4 matrix of JUST rotation parameters
	public Matrix getR(long frameNumber) {

		// Calculate this frame's rotation parameters
		float gamma = (float) this.rotX * (frameNumber + this.START_FRAME);
		float beta = (float) this.rotY * (frameNumber + this.START_FRAME);
		float alpha = (float) this.rotZ * (frameNumber + this.START_FRAME);

		Matrix Rx = Matrix.identity(4, 4);
		Rx.set(1, 1, Math.cos(gamma));
		Rx.set(2, 2, Math.cos(gamma));
		Rx.set(1, 2, -Math.sin(gamma));
		Rx.set(2, 1, Math.sin(gamma));

		Matrix Ry = Matrix.identity(4, 4);
		Ry.set(0, 0, Math.cos(beta));
		Ry.set(2, 2, Math.cos(beta));
		Ry.set(2, 0, -Math.sin(beta));
		Ry.set(0, 2, Math.sin(beta));

		Matrix Rz = Matrix.identity(4, 4);
		Rz.set(0, 0, Math.cos(alpha));
		Rz.set(1, 1, Math.cos(alpha));
		Rz.set(0, 1, -Math.sin(alpha));
		Rz.set(1, 0, Math.sin(alpha));

		return Rz.times(Ry).times(Rx);
	}

	// Returns homogeneous 4x4 matrix of WORLD translation parameters
	public Matrix getIC(long frameNumber) {
		// Calculate C
		Matrix C = Matrix.identity(4, 4);

		C.set(0, 3, this.initialTranslation.x + this.translationVelocity.x * (frameNumber + this.START_FRAME));
		C.set(1, 3, this.initialTranslation.y + this.translationVelocity.y * (frameNumber + this.START_FRAME));
		C.set(2, 3, this.initialTranslation.z + this.translationVelocity.z * (frameNumber + this.START_FRAME));

		return C;
	}

	public ArrayList<Point> getKeypoints(long frameNumber) {
		ArrayList<Point> keypoints = new ArrayList<Point>();

		Matrix R = this.getR(frameNumber);
		Matrix C = this.getIC(frameNumber);
		Matrix E = R.times(C);

		Matrix cameraMatrix = this.K.times(E.getMatrix(0, 2, 0, 3));

		for (int i = 0; i < this.worldCoordinates.size(); i++) {
			Matrix point = cameraMatrix.times(this.worldCoordinates.get(i));
			point = point.times(1 / point.get(2, 0));
			Point pt = new Point(Math.round(point.get(0, 0)), Math.round(point.get(1, 0)));
			keypoints.add(pt);
		}

		return keypoints;
	}

	public ArrayList<Matrix> getWorldCoordinates() {
		return worldCoordinates;
	}

	public void setWorldCoordinates(ArrayList<Matrix> worldCoordinates) {
		this.worldCoordinates = worldCoordinates;
	}

	public int getHEIGHT() {
		return HEIGHT;
	}

	public void setHEIGHT(int hEIGHT) {
		HEIGHT = hEIGHT;
	}

	public int getWIDTH() {
		return WIDTH;
	}

	public void setWIDTH(int wIDTH) {
		WIDTH = wIDTH;
	}

	public long getMAX_FRAMES() {
		return MAX_FRAMES;
	}

	public void setMAX_FRAMES(long mAX_FRAMES) {
		MAX_FRAMES = mAX_FRAMES;
	}

	public int getNUM_POINTS() {
		return NUM_POINTS;
	}

	public void setNUM_POINTS(int nUM_POINTS) {
		NUM_POINTS = nUM_POINTS;
	}

	public Matrix getK() {
		return K;
	}

	public void setK(Matrix k) {
		K = k;
	}

	public Vector3f getInitialTranslation() {
		return initialTranslation;
	}

	public void setInitialTranslation(Vector3f initialTranslation) {
		this.initialTranslation = initialTranslation;
	}

	public double getInitialRotX() {
		return initialRotX;
	}

	public void setInitialRotX(double initialRotX) {
		this.initialRotX = initialRotX;
	}

	public double getInitialRotY() {
		return initialRotY;
	}

	public void setInitialRotY(double initialRotY) {
		this.initialRotY = initialRotY;
	}

	public double getInitialRotZ() {
		return initialRotZ;
	}

	public void setInitialRotZ(double initialRotZ) {
		this.initialRotZ = initialRotZ;
	}

	public Vector3f getTranslationVelocity() {
		return translationVelocity;
	}

	public void setTranslationVelocity(Vector3f translationVelocity) {
		this.translationVelocity = translationVelocity;
	}

	public double getRotX() {
		return rotX;
	}

	public void setRotX(double rotX) {
		this.rotX = rotX;
	}

	public double getRotY() {
		return rotY;
	}

	public void setRotY(double rotY) {
		this.rotY = rotY;
	}

	public double getRotZ() {
		return rotZ;
	}

	public void setRotZ(double rotZ) {
		this.rotZ = rotZ;
	}
}
