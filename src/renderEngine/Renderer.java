package renderEngine;

import models.RawModel;
import models.TexturedModel;

import java.util.ArrayList;

import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL13;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL30;
import org.lwjgl.util.vector.Matrix4f;

import shaders.StaticShader;
import toolbox.Maths;
import entities.Camera;
import entities.Entity;

public class Renderer {
	
	private static final float FOV = 70;
	private static final float NEAR_PLANE = 0.1f;
	private static final float FAR_PLANE = 1000;
	
	private Matrix4f projectionMatrix;
	
	public Renderer(StaticShader shader){
		createProjectionMatrix();
		shader.start();
		shader.loadProjectionMatrix(projectionMatrix);
		shader.stop();
	}

	public void prepare() {
		
		GL11.glClear(GL11.GL_COLOR_BUFFER_BIT|GL11.GL_DEPTH_BUFFER_BIT);
		GL11.glClearColor(0, 0.0f, 0.0f, 1);
		
	}

	public void render(Camera camera, ArrayList<Entity> entities, StaticShader cameraShader, Entity background, StaticShader bgShader) {
		
		GL11.glDisable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_NEVER);
		bgShader.start();
		TexturedModel bgModel = background.getModel();
		RawModel bgRawModel = bgModel.getRawModel();
		GL30.glBindVertexArray(bgRawModel.getVaoID());
		GL20.glEnableVertexAttribArray(0);
		GL20.glEnableVertexAttribArray(1);
		GL13.glActiveTexture(GL13.GL_TEXTURE0);
		GL11.glBindTexture(GL11.GL_TEXTURE_2D, bgModel.getTexture().getID());
		GL11.glDrawElements(GL11.GL_TRIANGLES, bgRawModel.getVertexCount(), GL11.GL_UNSIGNED_INT, 0);
		GL20.glDisableVertexAttribArray(0);
		GL20.glDisableVertexAttribArray(1);
		GL30.glBindVertexArray(0);
		bgShader.stop();
		
		GL11.glEnable(GL11.GL_DEPTH_TEST);
		GL11.glDepthFunc(GL11.GL_LEQUAL);
		cameraShader.start();
		cameraShader.loadViewMatrix(camera);
		for (Entity entity:entities) {
			TexturedModel model = entity.getModel();
			RawModel rawModel = model.getRawModel();
			GL30.glBindVertexArray(rawModel.getVaoID());
			GL20.glEnableVertexAttribArray(0);
			GL20.glEnableVertexAttribArray(1);
			Matrix4f transformationMatrix = Maths.createTransformationMatrix(entity.getPosition(),
					entity.getRotX(), entity.getRotY(), entity.getRotZ(), entity.getScale());
			cameraShader.loadTransformationMatrix(transformationMatrix);
			GL13.glActiveTexture(GL13.GL_TEXTURE0);
			GL11.glBindTexture(GL11.GL_TEXTURE_2D, model.getTexture().getID());
			GL11.glDrawElements(GL11.GL_TRIANGLES, rawModel.getVertexCount(), GL11.GL_UNSIGNED_INT, 0);
			GL20.glDisableVertexAttribArray(0);
			GL20.glDisableVertexAttribArray(1);
			GL30.glBindVertexArray(0);
		}
		cameraShader.stop();
	}
	
	private void createProjectionMatrix(){
		float aspectRatio = (float) Display.getWidth() / (float) Display.getHeight();
		float y_scale = (float) ((1f / Math.tan(Math.toRadians(FOV / 2f))) * aspectRatio);
		float x_scale = y_scale / aspectRatio;
		float frustum_length = FAR_PLANE - NEAR_PLANE;


		projectionMatrix = new Matrix4f();
		projectionMatrix.m00 = x_scale;
		projectionMatrix.m11 = y_scale;
		projectionMatrix.m22 = -((FAR_PLANE + NEAR_PLANE) / frustum_length);
		projectionMatrix.m23 = -1;
		projectionMatrix.m32 = -((2 * NEAR_PLANE * FAR_PLANE) / frustum_length);
		projectionMatrix.m33 = 0;
		

				
//		K.put(0, 0, 527.0593);
//		K.put(0,  1, -0.9513);
//		K.put(0,  2, 240.0892);
//		K.put(1, 0, 0.0);
//		K.put(1, 1, 488.0588);
//		K.put(1, 2, 168.4744);
//		K.put(2,  0, 0.0);
//		K.put(2,  1, 0.0);
//		K.put(2,  2, 1.0);
		
		projectionMatrix.m00 = 527.0593f;
		projectionMatrix.m01 = -0.9513f;
		projectionMatrix.m02 = 240.0892f;
		projectionMatrix.m10 = 0.0f;
		projectionMatrix.m11 = 488.0588f;
		projectionMatrix.m12 = 168.4744f;
		projectionMatrix.m20 = 0.0f;
		projectionMatrix.m21 = 0.0f;
		projectionMatrix.m22 = 1.0f;
		
		projectionMatrix.m03 = 0.0f;
		projectionMatrix.m13 = 0.0f;
		projectionMatrix.m23 = 0.0f;
		
		projectionMatrix.m30 = 0.0f;
		projectionMatrix.m31 = 0.0f;
		projectionMatrix.m32 = 0.0f;
		projectionMatrix.m33 = 1.0f;
	}

}
