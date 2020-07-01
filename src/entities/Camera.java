package entities;

import org.lwjgl.input.Keyboard;
import org.lwjgl.util.vector.Vector3f;

public class Camera {
	
	private Vector3f position = new Vector3f(0,0,0);
	private float pitch;
	private float yaw;
	private float roll;
	
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
	
	

}
