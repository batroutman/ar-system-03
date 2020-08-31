
import java.nio.ByteBuffer;
import java.util.ArrayList;

import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL30;
import org.lwjgl.util.vector.Vector3f;

import ARPipeline.Frame;
import ARPipeline.FrameBuffer;
import ARPipeline.Pose;
import ARPipeline.PoseBuffer;
import entities.Camera;
import entities.Entity;
import models.RawModel;
import models.TexturedModel;
import renderEngine.DisplayManager;
import renderEngine.Loader;
import renderEngine.Renderer;
import shaders.StaticShader;
import textures.ModelTexture;

public class OpenGLARDisplay {

	PoseBuffer poseBuffer;
	FrameBuffer frameBuffer;

	Loader loader;
	Renderer renderer;
	Camera camera;
	ArrayList<Entity> entities = new ArrayList<Entity>();
	StaticShader cameraShader;
	Entity bgEntity;
	StaticShader bgShader;

	public OpenGLARDisplay() {
		this.initOpenGL();
	}

	public OpenGLARDisplay(FrameBuffer frameBuffer, PoseBuffer poseBuffer) {
		this.frameBuffer = frameBuffer;
		this.poseBuffer = poseBuffer;
		this.initOpenGL();
	}

	public void initOpenGL() {

		// initialize
		DisplayManager.createDisplay();
		this.loader = new Loader();
		this.cameraShader = new StaticShader(false);
		this.renderer = new Renderer(this.cameraShader);

		// temporarily set up cube data (eventually load from blender)
		float[] vertices = { -0.5f, 0.5f, 0, -0.5f, -0.5f, 0, 0.5f, -0.5f, 0, 0.5f, 0.5f, 0,

				-0.5f, 0.5f, 1, -0.5f, -0.5f, 1, 0.5f, -0.5f, 1, 0.5f, 0.5f, 1,

				0.5f, 0.5f, 0, 0.5f, -0.5f, 0, 0.5f, -0.5f, 1, 0.5f, 0.5f, 1,

				-0.5f, 0.5f, 0, -0.5f, -0.5f, 0, -0.5f, -0.5f, 1, -0.5f, 0.5f, 1,

				-0.5f, 0.5f, 1, -0.5f, 0.5f, 0, 0.5f, 0.5f, 0, 0.5f, 0.5f, 1,

				-0.5f, -0.5f, 1, -0.5f, -0.5f, 0, 0.5f, -0.5f, 0, 0.5f, -0.5f, 1

		};

		float[] textureCoords = {

				0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0,
				1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0

		};

		int[] indices = { 0, 1, 3, 3, 1, 2, 4, 5, 7, 7, 5, 6, 8, 9, 11, 11, 9, 10, 12, 13, 15, 15, 13, 14, 16, 17, 19,
				19, 17, 18, 20, 21, 23, 23, 21, 22

		};

		RawModel tModel = this.loader.loadToVAO(vertices, textureCoords, indices);
		TexturedModel tStaticModel = new TexturedModel(tModel,
				new ModelTexture(this.loader.loadTexture("sample_texture_128")));
		Entity tEntity = new Entity(tStaticModel, new Vector3f(0, 0, 1000), 0, 0, 0, 100f);
		this.entities.add(tEntity);

		// create camera
		this.camera = new Camera();

		// create background data
		float[] bgVertices = { -1, 1, 0, -1, -1, 0, 1, -1, 0, 1, 1, 0 };

		int[] bgIndices = { 0, 1, 3, 3, 1, 2 };

		float[] bgTextureCoords = { 0, 0, 0, 1, 1, 1, 1, 0 };

		// set up background models
		this.bgShader = new StaticShader(true);
		RawModel bgModel = this.loader.loadToVAO(bgVertices, bgTextureCoords, bgIndices);
		TexturedModel bgStaticModel = new TexturedModel(bgModel,
				new ModelTexture(this.loader.loadTexture("sample_texture")));
		this.bgEntity = new Entity(bgStaticModel, new Vector3f(0, 0, -10), 0, 0, 0, 2000);

	}

	public void updateDisplay() {
		this.renderer.prepare();
		this.renderer.render(this.camera, this.entities, this.cameraShader, this.bgEntity, this.bgShader);
		DisplayManager.updateDisplay();
	}

	public void setFrameToTexture(Frame frame) {
		// convert Frame to texture
		byte[] bytes = new byte[frame.getGrey().length * 3];
		if (frame.getR() != null) {
			for (int i = 0; i < bytes.length / 3; i++) {
				byte R = frame.getR()[i];
				byte G = frame.getG()[i];
				byte B = frame.getB()[i];
				bytes[i * 3] = R;
				bytes[i * 3 + 1] = G;
				bytes[i * 3 + 2] = B;
			}
		} else {
			for (int i = 0; i < bytes.length; i++) {
				bytes[i] = frame.getGrey()[i / 3];
			}
		}

		ByteBuffer pixels = ByteBuffer.allocateDirect(bytes.length);
		pixels.put(bytes);
		pixels.flip();

		// delete old texture and create new texture
		GL11.glDeleteTextures(this.bgEntity.getModel().getTexture().getID());
		int textureID = GL11.glGenTextures();
		GL11.glBindTexture(GL11.GL_TEXTURE_2D, textureID);
		GL11.glPixelStorei(GL11.GL_UNPACK_ALIGNMENT, 1);
		GL11.glTexParameterf(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MIN_FILTER, GL11.GL_LINEAR);
		GL11.glTexParameterf(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MAG_FILTER, GL11.GL_LINEAR);
		GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, GL11.GL_RGB, frame.getWidth(), frame.getHeight(), 0, GL11.GL_RGB,
				GL11.GL_UNSIGNED_BYTE, pixels);
		GL30.glGenerateMipmap(GL11.GL_TEXTURE_2D);

		// set bgEntity texture to new texture
		this.bgEntity.getModel().getTexture().setID(textureID);
	}

	public void setCameraPose(Pose pose) {
		this.camera.setMatrix(pose);
	}

	public void detectChanges() {

		Pose pose;
		Frame frame;
		synchronized (this.poseBuffer) {
			pose = this.poseBuffer.getCurrentPose();
		}
		synchronized (this.frameBuffer) {
			frame = this.frameBuffer.getCurrentFrame();
		}
		if (pose != null) {
			this.setCameraPose(pose);
			// DEBUG LINE
			// this.camera.move();
		}
		if (frame != null) {
			this.setFrameToTexture(frame);
		}
	}

	public void displayLoop() {

		while (!Display.isCloseRequested()) {
			this.detectChanges();
			this.updateDisplay();
		}
		this.cameraShader.cleanUp();
		this.loader.cleanUp();
		DisplayManager.closeDisplay();

	}

}
