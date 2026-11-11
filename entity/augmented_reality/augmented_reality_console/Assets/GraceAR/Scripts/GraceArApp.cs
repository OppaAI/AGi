using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.EnhancedTouch;
using UnityEngine.InputSystem.XR;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.ARSubsystems;
using Unity.XR.CoreUtils;
using Touch = UnityEngine.InputSystem.EnhancedTouch.Touch;
using TouchPhase = UnityEngine.InputSystem.TouchPhase;

namespace Aiva.GraceAR
{
    public sealed class GraceArApp : MonoBehaviour
    {
        private static readonly List<ARRaycastHit> Hits = new();

        private ARRaycastManager _raycastManager;
        private ARPlaneManager _planeManager;
        private GraceChamber _chamber;
        private GraceVrmLoader _loader;
        private string _status = "Move the phone slowly so ARCore can find the floor.";

        private void Awake()
        {
            EnhancedTouchSupport.Enable();
            CreateArRig();
            _chamber = GraceChamber.Create();
            _chamber.gameObject.SetActive(false);

            _loader = gameObject.AddComponent<GraceVrmLoader>();
            _loader.Loaded += model =>
            {
                _chamber.Actions.Bind(model);
                _status = "Grace is ready. Tap a floor point to place her chamber.";
            };
            _loader.Failed += error => _status = error;
            _loader.LoadInto(_chamber.ModelMount);
        }

        private void CreateArRig()
        {
            var sessionObject = new GameObject("AR Session");
            sessionObject.AddComponent<ARSession>();
            sessionObject.AddComponent<ARInputManager>();

            var originObject = new GameObject("XR Origin");
            var origin = originObject.AddComponent<XROrigin>();
            _raycastManager = originObject.AddComponent<ARRaycastManager>();
            _planeManager = originObject.AddComponent<ARPlaneManager>();
            originObject.AddComponent<ARAnchorManager>();

            var offset = new GameObject("Camera Offset");
            offset.transform.SetParent(originObject.transform, false);
            var cameraObject = new GameObject("AR Camera");
            cameraObject.transform.SetParent(offset.transform, false);
            cameraObject.tag = "MainCamera";
            var camera = cameraObject.AddComponent<Camera>();
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = Color.black;
            cameraObject.AddComponent<AudioListener>();
            var poseDriver = cameraObject.AddComponent<TrackedPoseDriver>();
            var positionAction = new InputAction("AR camera position", InputActionType.Value, "<XRHMD>/centerEyePosition");
            var rotationAction = new InputAction("AR camera rotation", InputActionType.Value, "<XRHMD>/centerEyeRotation");
            positionAction.Enable();
            rotationAction.Enable();
            poseDriver.positionInput = new InputActionProperty(positionAction);
            poseDriver.rotationInput = new InputActionProperty(rotationAction);
            cameraObject.AddComponent<ARCameraManager>();
            cameraObject.AddComponent<ARCameraBackground>();
            origin.CameraFloorOffsetObject = offset;
            origin.Camera = camera;
        }

        private void Update()
        {
            if (Touch.activeTouches.Count == 0 || Touch.activeTouches[0].phase != TouchPhase.Began) return;
            var touch = Touch.activeTouches[0];
            if (touch.screenPosition.y < 180f) return;
            PlaceAt(touch.screenPosition);
        }

        private void OnDestroy()
        {
            if (EnhancedTouchSupport.enabled) EnhancedTouchSupport.Disable();
        }

        private void PlaceAt(Vector2 screenPoint)
        {
            if (!_raycastManager.Raycast(screenPoint, Hits, TrackableType.PlaneWithinPolygon))
            {
                _status = "No floor found there. Move the phone slowly and try again.";
                return;
            }

            var pose = Hits[0].pose;
            var camera = Camera.main;
            var towardCamera = camera == null ? Vector3.forward : camera.transform.position - pose.position;
            towardCamera.y = 0f;
            if (towardCamera.sqrMagnitude > 0.001f)
            {
                pose.rotation = Quaternion.LookRotation(towardCamera.normalized, Vector3.up);
            }

            _chamber.transform.SetPositionAndRotation(pose.position, pose.rotation);
            _chamber.gameObject.SetActive(true);
            _status = "Grace’s chamber is anchored here for this AR session.";
            _planeManager.enabled = false;
        }

        private void OnGUI()
        {
            var scale = Mathf.Max(1f, Screen.dpi / 180f);
            var style = new GUIStyle(GUI.skin.box)
            {
                fontSize = Mathf.RoundToInt(16f * scale),
                alignment = TextAnchor.MiddleLeft,
                wordWrap = true,
                padding = new RectOffset(18, 18, 12, 12)
            };
            GUI.Box(new Rect(16, 16, Screen.width - 32, 76 * scale), _status, style);

            var buttonWidth = (Screen.width - 48) / 3f;
            var buttonHeight = 56f * scale;
            var y = Screen.height - buttonHeight - 20f;
            if (GUI.Button(new Rect(12, y, buttonWidth, buttonHeight), "Place / Move"))
            {
                _planeManager.enabled = true;
                _status = "Tap the floor where the chamber should stand.";
            }
            if (GUI.Button(new Rect(24 + buttonWidth, y, buttonWidth, buttonHeight), "Wave"))
            {
                _chamber.Actions.Wave();
            }
            if (GUI.Button(new Rect(36 + buttonWidth * 2, y, buttonWidth, buttonHeight), "Turn"))
            {
                _chamber.Actions.TurnAround();
            }
        }
    }
}

