package entities;

import org.lwjgl.input.Keyboard;
import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;

import ARPipeline.Pose;

public class Camera {

	private Vector3f position = new Vector3f(0, 0, 0);
	private float pitch;
	private float yaw;
	private float roll;

	// extrinsic format
	float r00;
	float r01;
	float r02;
	float tx;
	float r10;
	float r11;
	float r12;
	float ty;
	float r20;
	float r21;
	float r22;
	float tz;

	public Camera() {
	}

	float rotAngle = 0;

	public void move() {
		if (Keyboard.isKeyDown(Keyboard.KEY_W)) {
			System.out.println("W PRESSED");
			float magnitude = 0.001f;
			tz += magnitude * Math.sin(Math.toRadians(yaw - 90));
			tx += magnitude * Math.cos(Math.toRadians(yaw - 90));
		}
		if (Keyboard.isKeyDown(Keyboard.KEY_D)) {
			rotAngle += 0.01f;
			this.r11 = ((float) Math.cos(rotAngle));
			this.r22 = ((float) Math.cos(rotAngle));
			this.r12 = (-(float) Math.sin(rotAngle));
			this.r21 = ((float) Math.sin(rotAngle));
		}
		if (Keyboard.isKeyDown(Keyboard.KEY_A)) {
			rotAngle -= 0.01f;
			this.r11 = ((float) Math.cos(rotAngle));
			this.r22 = ((float) Math.cos(rotAngle));
			this.r12 = (-(float) Math.sin(rotAngle));
			this.r21 = ((float) Math.sin(rotAngle));
		}
		if (Keyboard.isKeyDown(Keyboard.KEY_S)) {
			System.out.println("S PRESSED");
			float magnitude = 0.001f;
			tz -= magnitude * Math.sin(Math.toRadians(yaw - 90));
			tx -= magnitude * Math.cos(Math.toRadians(yaw - 90));
		}
	}

	public Vector3f getPosition() {
		return position;
	}

	public float getPitch() {
		return pitch;
	}

	public float getYaw() {
		return yaw;
	}

	public float getRoll() {
		return roll;
	}

	public void setMatrix(Pose pose) {
		this.r00 = (float) pose.getR00();
		this.r01 = (float) pose.getR01();
		this.r02 = (float) pose.getR02();
		this.r10 = (float) pose.getR10();
		this.r11 = (float) pose.getR11();
		this.r12 = (float) pose.getR12();
		this.r20 = (float) pose.getR20();
		this.r21 = (float) pose.getR21();
		this.r22 = (float) pose.getR22();
		this.tx = (float) pose.getTx();
		this.ty = (float) pose.getTy();
		this.tz = (float) pose.getTz();

	}

	public Matrix4f getViewMatrix() {
		Matrix4f rotationOffset = new Matrix4f();
		rotationOffset.setIdentity();
		// rotationOffset.m00 = -1;
		// rotationOffset.m11 = -1;
		// rotationOffset.m22 = -1;
		// Matrix4f.rotate((float) Math.PI, new Vector3f(1, 0, 0),
		// rotationOffset, rotationOffset);
		// Matrix4f.rotate((float) Math.PI, new Vector3f(0, 0, 1),
		// rotationOffset, rotationOffset);

		Matrix4f mat = new Matrix4f();
		mat.setIdentity();
		mat.m00 = r00;
		mat.m01 = r01;
		mat.m02 = r02;
		mat.m10 = r10;
		mat.m11 = r11;
		mat.m12 = r12;
		mat.m20 = r20;
		mat.m21 = r21;
		mat.m22 = r22;

		Matrix4f.translate(new Vector3f(-tx, -ty, -tz), mat, mat);

		Matrix4f result = new Matrix4f();
		Matrix4f.mul(mat, rotationOffset, result);

		return result;
	}

}
