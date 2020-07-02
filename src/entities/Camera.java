package entities;

import org.lwjgl.input.Keyboard;
import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;

import ARPipeline.Pose;

public class Camera {
	
	private Vector3f position = new Vector3f(0,0,0);
	private float pitch;
	private float yaw;
	private float roll;
	
	// extrinsic format
	float r00; float r01; float r02; float tx;
	float r10; float r11; float r12; float ty;
	float r20; float r21; float r22; float tz;
	
	public Camera(){}
	
	public void move(){
		if(Keyboard.isKeyDown(Keyboard.KEY_W)){
			float magnitude = 0.1f;
			position.z+=magnitude * Math.sin(Math.toRadians(yaw - 90));
			position.x+=magnitude * Math.cos(Math.toRadians(yaw - 90));
		}
		if(Keyboard.isKeyDown(Keyboard.KEY_D)){
			yaw+=0.5f;
		}
		if(Keyboard.isKeyDown(Keyboard.KEY_A)){
			yaw-=0.5f;
		}
		if(Keyboard.isKeyDown(Keyboard.KEY_S)){
			float magnitude = 0.1f;
			position.z-=magnitude * Math.sin(Math.toRadians(yaw - 90));
			position.x-=magnitude * Math.cos(Math.toRadians(yaw - 90));
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
		this.r00 = pose.getR00();
		this.r01 = pose.getR01();
		this.r02 = pose.getR02();
		this.r10 = pose.getR10();
		this.r11 = pose.getR11();
		this.r12 = pose.getR12();
		this.r20 = pose.getR20();
		this.r21 = pose.getR21();
		this.r22 = pose.getR22();
		this.tx = pose.getTx();
		this.ty = pose.getTy();
		this.tz = pose.getTz();
		
	}
	
	public Matrix4f getViewMatrix() {
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
		mat.m03 = tx;
		mat.m13 = ty;
		mat.m23 = tz;
		return mat;
	}
	

}
