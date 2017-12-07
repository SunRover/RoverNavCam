/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.projecttango.examples.java.pointcloud;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;

import android.content.Context;
import android.graphics.Color;
import android.util.Log;
import android.view.MotionEvent;

import org.rajawali3d.materials.Material;
import org.rajawali3d.materials.textures.ATexture;
import org.rajawali3d.materials.textures.StreamingTexture;
import org.rajawali3d.math.Matrix4;
import org.rajawali3d.math.Quaternion;
import org.rajawali3d.math.vector.Vector3;
import org.rajawali3d.primitives.ScreenQuad;
import org.rajawali3d.renderer.RajawaliRenderer;

import com.google.tango.support.TangoSupport;
import com.projecttango.examples.java.pointcloud.rajawali.FrustumAxes;
import com.projecttango.examples.java.pointcloud.rajawali.Grid;
import com.projecttango.examples.java.pointcloud.rajawali.PointCloud;

import javax.microedition.khronos.opengles.GL10;

/**
 * Renderer for Point Cloud data.
 */
public class PointCloudRajawaliRenderer extends RajawaliRenderer {
    private static final String TAG = PointCloudRajawaliRenderer.class.getSimpleName();

    private static final float CAMERA_NEAR = 0.01f;
    private static final float CAMERA_FAR = 200f;
    private static final int MAX_NUMBER_OF_POINTS = 60000;

    private TouchViewHandler mTouchViewHandler;

    private float[] textureCoords0 = new float[]{0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.0F};

    // Rajawali texture used to render the Tango color camera.
    private ATexture mTangoCameraTexture;

    // Keeps track of whether the scene camera has been configured.
    private boolean mSceneCameraConfigured;

    private ScreenQuad mBackgroundQuad;

    // Objects rendered in the scene.
    private PointCloud mPointCloud;
    private FrustumAxes mFrustumAxes;
    private Grid mGrid;

    double yaw = 0;
    float[] rotation;
    float[] translation;

    public PointCloudRajawaliRenderer(Context context) {
        super(context);
        mTouchViewHandler = new TouchViewHandler(mContext, getCurrentCamera());
    }

    @Override
    protected void initScene() {

        // Create a quad covering the whole background and assign a texture to it where the
        // Tango color camera contents will be rendered.
        Material tangoCameraMaterial = new Material();
        tangoCameraMaterial.setColorInfluence(0);

        if (mBackgroundQuad == null) {
            mBackgroundQuad = new ScreenQuad();
            mBackgroundQuad.getGeometry().setTextureCoords(textureCoords0);
        }
        // We need to use Rajawali's {@code StreamingTexture} since it sets up the texture
        // for GL_TEXTURE_EXTERNAL_OES rendering.
        mTangoCameraTexture =
                new StreamingTexture("camera", (StreamingTexture.ISurfaceListener) null);
        try {
            tangoCameraMaterial.addTexture(mTangoCameraTexture);
            mBackgroundQuad.setMaterial(tangoCameraMaterial);
        } catch (ATexture.TextureException e) {
            Log.e(TAG, "Exception creating texture for RGB camera contents", e);
        }
        getCurrentScene().addChildAt(mBackgroundQuad, 0);

        mGrid = new Grid(100, 1, 1, 0xFFCCCCCC);
        mGrid.setPosition(0, -1.3f, 0);
        getCurrentScene().addChild(mGrid);

        mFrustumAxes = new FrustumAxes(3);
        getCurrentScene().addChild(mFrustumAxes);

        // Indicate four floats per point since the point cloud data comes
        // in XYZC format.
        mPointCloud = new PointCloud(MAX_NUMBER_OF_POINTS, 4);
        getCurrentScene().addChild(mPointCloud);
        getCurrentScene().setBackgroundColor(Color.WHITE);
        getCurrentCamera().setNearPlane(CAMERA_NEAR);
        getCurrentCamera().setFarPlane(CAMERA_FAR);
        getCurrentCamera().setFieldOfView(37.5);
    }

    /**
     * Update background texture's UV coordinates when device orientation is changed (i.e., change
     * between landscape and portrait mode).
     * This must be run in the OpenGL thread.
     */
    public void updateColorCameraTextureUvGlThread(int rotation) {
        if (mBackgroundQuad == null) {
            mBackgroundQuad = new ScreenQuad();
        }

        float[] textureCoords =
                TangoSupport.getVideoOverlayUVBasedOnDisplayRotation(textureCoords0, rotation);
        mBackgroundQuad.getGeometry().setTextureCoords(textureCoords);
        mBackgroundQuad.getGeometry().reload();
    }

    /**
     * Update the scene camera based on the provided pose in Tango start of service frame.
     * The camera pose should match the pose of the camera color at the time of the last rendered
     * RGB frame, which can be retrieved with this.getTimestamp();
     * <p/>
     * NOTE: This must be called from the OpenGL render thread; it is not thread-safe.
     */
    public void updateRenderCameraPose(TangoPoseData cameraPose) {
        float[] rotation = cameraPose.getRotationAsFloats();
        float[] translation = cameraPose.getTranslationAsFloats();
        Quaternion quaternion = new Quaternion(rotation[3], rotation[0], rotation[1], rotation[2]);
        // Conjugating the Quaternion is needed because Rajawali uses left-handed convention for
        // quaternions.
        getCurrentCamera().setRotation(quaternion.conjugate());
        getCurrentCamera().setPosition(translation[0], translation[1], translation[2]);
    }

    /**
     * It returns the ID currently assigned to the texture where the Tango color camera contents
     * should be rendered.
     * NOTE: This must be called from the OpenGL render thread; it is not thread-safe.
     */
    public int getTextureId() {
        return mTangoCameraTexture == null ? -1 : mTangoCameraTexture.getTextureId();
    }

    /**
     * We need to override this method to mark the camera for re-configuration (set proper
     * projection matrix) since it will be reset by Rajawali on surface changes.
     */
    @Override
    public void onRenderSurfaceSizeChanged(GL10 gl, int width, int height) {
        super.onRenderSurfaceSizeChanged(gl, width, height);
        mSceneCameraConfigured = false;
    }

    public boolean isSceneCameraConfigured() {
        return mSceneCameraConfigured;
    }

    /**
     * Sets the projection matrix for the scene camera to match the parameters of the color camera,
     * provided by the {@code TangoCameraIntrinsics}.
     */
    public void setProjectionMatrix(float[] matrixFloats) {
        getCurrentCamera().setProjectionMatrix(new Matrix4(matrixFloats));
    }

    @Override
    public void onOffsetsChanged(float xOffset, float yOffset,
                                 float xOffsetStep, float yOffsetStep,
                                 int xPixelOffset, int yPixelOffset) {
    }

    @Override
    public void onTouchEvent(MotionEvent event) {
        if (event.getAction() == MotionEvent.ACTION_DOWN) {
            Log.d(TAG, "Pick object attempt");
        }
    }


    /**
     * Updates the rendered point cloud. For this, we need the point cloud data and the device pose
     * at the time the cloud data was acquired.
     * NOTE: This needs to be called from the OpenGL rendering thread.
     */
    public void updatePointCloud(TangoPointCloudData pointCloudData, float[] openGlTdepth) {
        mPointCloud.updateCloud(pointCloudData.numPoints, pointCloudData.points);
        Matrix4 openGlTdepthMatrix = new Matrix4(openGlTdepth);
        openGlTdepthMatrix.rotate(Vector3.Axis.X, 13);
        openGlTdepthMatrix.rotate(Vector3.Axis.Y, 2);
//        openGlTdepthMatrix.translate(-0.065, 0, 0);
//        mPointCloud.setPosition(openGlTdepthMatrix.getTranslation().x - 0.065, openGlTdepthMatrix.getTranslation().y, openGlTdepthMatrix.getTranslation().z);
        mPointCloud.setPosition(openGlTdepthMatrix.getTranslation());
        // Conjugating the Quaternion is needed because Rajawali uses left-handed convention.
        mPointCloud.setOrientation(new Quaternion().fromMatrix(openGlTdepthMatrix).conjugate());
    }

    /**
     * Updates our information about the current device pose.
     * NOTE: This needs to be called from the OpenGL rendering thread.
     */

    public void updateCameraPose(TangoPoseData cameraPose) {
        rotation = cameraPose.getRotationAsFloats();
        // roll (x-axis rotation)
//        double sinr = +2.0 * (rotation[cameraPose.INDEX_ROTATION_W] * rotation[cameraPose.INDEX_ROTATION_X] + rotation[cameraPose.INDEX_ROTATION_Y] * rotation[cameraPose.INDEX_ROTATION_Z]);
//        double cosr = +1.0 - 2.0 * (rotation[cameraPose.INDEX_ROTATION_X] * rotation[cameraPose.INDEX_ROTATION_X] + rotation[cameraPose.INDEX_ROTATION_Y] * rotation[cameraPose.INDEX_ROTATION_Y]);
//        double roll = Math.atan2(sinr, cosr);
//
//        // pitch (y-axis rotation)
//        double pitch;
//        double sinp = +2.0 * (rotation[cameraPose.INDEX_ROTATION_W] * rotation[cameraPose.INDEX_ROTATION_Y] - rotation[cameraPose.INDEX_ROTATION_Z] * rotation[cameraPose.INDEX_ROTATION_X]);
//            pitch = Math.asin(sinp);
//
//        // yaw (z-axis rotation)
//        double siny = +2.0 * (rotation[cameraPose.INDEX_ROTATION_W] * rotation[cameraPose.INDEX_ROTATION_Z] + rotation[cameraPose.INDEX_ROTATION_X] * rotation[cameraPose.INDEX_ROTATION_Y]);
//        double cosy = +1.0 - 2.0 * (rotation[cameraPose.INDEX_ROTATION_Y] * rotation[cameraPose.INDEX_ROTATION_Y] + rotation[cameraPose.INDEX_ROTATION_Z] * rotation[cameraPose.INDEX_ROTATION_Z]);
//        double yaw = Math.atan2(siny, cosy);
//        Log.d(TAG, Math.toDegrees(roll) + " " + Math.toDegrees(pitch) + " " + Math.toDegrees(yaw));
        translation = cameraPose.getTranslationAsFloats();
//        translation[0] += 0.065;
//        Log.d(TAG, translation[cameraPose.INDEX_TRANSLATION_X] + " " + translation[cameraPose.INDEX_TRANSLATION_Y] + " " + translation[cameraPose.INDEX_TRANSLATION_Z]);
        Quaternion quaternion = new Quaternion(rotation[3], rotation[0], rotation[1], rotation[2]);
        mFrustumAxes.setPosition(translation[0], translation[1], translation[2]);

        //Log.d("translation", translation[0] + " " + translation[1] + " " + translation[2]);
        yaw = quaternion.getYaw();

        // Conjugating the Quaternion is needed because Rajawali uses left-handed convention for
        // quaternions.
        mFrustumAxes.setOrientation(quaternion.conjugate());
        mTouchViewHandler.updateCamera(new Vector3(translation[0], translation[1], translation[2]),
                quaternion);
    }

    public double getYaw(){
        return yaw;
    }

    public float[] getTranslation(){
        return translation;
    }


    public void setFirstPersonView() {
        mTouchViewHandler.setFirstPersonView();
    }

    public void setTopDownView() {
        mTouchViewHandler.setTopDownView();
    }

    public void setThirdPersonView() {
        mTouchViewHandler.setThirdPersonView();
    }
}
