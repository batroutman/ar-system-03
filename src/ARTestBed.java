import models.RawModel;
import models.TexturedModel;

import org.lwjgl.opengl.Display;
import org.lwjgl.util.vector.Vector3f;

import renderEngine.DisplayManager;
import renderEngine.Loader;
import renderEngine.Renderer;
import shaders.StaticShader;
import textures.ModelTexture;
import entities.Camera;
import entities.Entity;

public class ARTestBed {

	public static void main(String[] args) {

		ARSetup arSetup = new ARSetup();
		arSetup.start();
		
//		while(!Display.isCloseRequested()){
//			camera.move();
//			renderer.prepare();
//			renderer.render(camera, entities, cameraShader, bgEntity, bgShader);
//			DisplayManager.updateDisplay();
//		}

//		cameraShader.cleanUp();
//		loader.cleanUp();
//		DisplayManager.closeDisplay();

	}

}
