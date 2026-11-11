using System;
using System.Collections.Generic;
using System.Threading.Tasks;
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

        [SerializeField] private XRReferenceImageLibrary markerLibrary;
        private ARAnchorManager _anchorManager;
        private ARAnchor _anchor;
        private ARTrackedImageManager _imageManager;
        private ARTrackedImage _marker;
        private GraceConnection _connection;
        private InputAction _positionAction, _rotationAction;
        private bool _placing, _placementBusy, _restored;
        private string _gateway, _prompt = "";
        private Vector2 _responseScroll;
        private const string MarkerPosition = "grace-ar-marker-position";
        [Serializable] private struct SavedPosition { public Vector3 position; public Quaternion rotation; }
        private ARRaycastManager _raycastManager;
        private ARPlaneManager _planeManager;
        private GraceChamber _chamber;
        private GraceVrmLoader _loader;
        private string _status = "Move the phone slowly so ARCore can find the floor.";

        private void Awake()
        {
            EnhancedTouchSupport.Enable();
            _gateway = PlayerPrefs.GetString("grace-ar-gateway", "http://YOUR-AIVA-PC:8853");
            _connection = gameObject.AddComponent<GraceConnection>();
            _placing = !PlayerPrefs.HasKey(MarkerPosition);
            CreateArRig();
            _chamber = GraceChamber.Create();
            _chamber.gameObject.SetActive(false);

            _loader = gameObject.AddComponent<GraceVrmLoader>();
            _loader.Loaded += model =>
            {
                _chamber.Actions.Bind(model);
                _status = _placing ? "Grace is ready. Tap a floor to place her chamber." : "Scan the printed Grace marker to restore her chamber.";
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
            _anchorManager = originObject.AddComponent<ARAnchorManager>();
            _planeManager.requestedDetectionMode = PlaneDetectionMode.Horizontal;
            if (markerLibrary != null)
            {
                _imageManager = originObject.AddComponent<ARTrackedImageManager>();
                _imageManager.referenceLibrary = markerLibrary;
                _imageManager.trackablesChanged.AddListener(OnMarkersChanged);
            }
            var light = new GameObject("Grace Key Light").AddComponent<Light>();
            light.type = LightType.Directional; light.intensity = 1f;
            light.transform.rotation = Quaternion.Euler(35f, -30f, 0f);

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
            _positionAction = new InputAction("AR camera position", InputActionType.Value, "<XRHMD>/centerEyePosition");
            _rotationAction = new InputAction("AR camera rotation", InputActionType.Value, "<XRHMD>/centerEyeRotation");
            _positionAction.Enable();
            _rotationAction.Enable();
            poseDriver.positionInput = new InputActionProperty(_positionAction);
            poseDriver.rotationInput = new InputActionProperty(_rotationAction);
            poseDriver.ignoreTrackingState = true;
            cameraObject.AddComponent<ARCameraManager>();
            cameraObject.AddComponent<ARCameraBackground>();
            origin.CameraFloorOffsetObject = offset;
            origin.Camera = camera;
        }

        private void Update()
        {
            if (!_placing || _placementBusy || Touch.activeTouches.Count == 0 || Touch.activeTouches[0].phase != TouchPhase.Began) return;
            var touch = Touch.activeTouches[0];
            var scale = UiScale;
            if (touch.screenPosition.y < 260f * scale || touch.screenPosition.y > Screen.height - 150f * scale) return;
            PlaceAt(touch.screenPosition);
        }

        private void OnDestroy()
        {
            _positionAction?.Dispose(); _rotationAction?.Dispose();
            if (EnhancedTouchSupport.enabled) EnhancedTouchSupport.Disable();
        }

        private async void PlaceAt(Vector2 screenPoint)
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

            _placementBusy = true;
            try
            {
                if (!await AttachChamber(pose)) return;
                if (_marker != null && _marker.trackingState == TrackingState.Tracking)
                {
                    var saved = new SavedPosition { position = _marker.transform.InverseTransformPoint(pose.position), rotation = Quaternion.Inverse(_marker.transform.rotation) * pose.rotation };
                    PlayerPrefs.SetString(MarkerPosition, JsonUtility.ToJson(saved)); PlayerPrefs.Save();
                    _status = "Placement saved. Scan the same marker after reopening to restore Grace.";
                }
                else
                {
                    PlayerPrefs.DeleteKey(MarkerPosition); PlayerPrefs.Save();
                    _status = "Session anchor placed. Scan the marker before placing to remember this spot.";
                }
                _restored = true;
                _chamber.gameObject.SetActive(true); _placing = false;
            }
            catch (Exception error) { _status = "Placement failed: " + error.Message; }
            finally { _placementBusy = false; }
        }

        private void OnMarkersChanged(ARTrackablesChangedEventArgs<ARTrackedImage> changes)
        {
            foreach (var image in changes.added) ObserveMarker(image);
            foreach (var image in changes.updated) ObserveMarker(image);
        }

        private async void ObserveMarker(ARTrackedImage image)
        {
            if (image.trackingState != TrackingState.Tracking) return;
            _marker = image;
            if (_placing || _placementBusy || _restored || !PlayerPrefs.HasKey(MarkerPosition)) return;
            _placementBusy = true;
            try
            {
                var saved = JsonUtility.FromJson<SavedPosition>(PlayerPrefs.GetString(MarkerPosition));
                var pose = new Pose(image.transform.TransformPoint(saved.position), image.transform.rotation * saved.rotation);
                if (await AttachChamber(pose))
                {
                    _restored = true; _chamber.gameObject.SetActive(true);
                    _status = "Chamber restored and anchored. You can look away from the marker now.";
                }
            }
            catch (Exception error) { _status = "Restore failed: " + error.Message; }
            finally { _placementBusy = false; }
        }

        private async Task<bool> AttachChamber(Pose pose)
        {
            var result = await _anchorManager.TryAddAnchorAsync(pose);
            if (!result.status.IsSuccess()) { _status = "ARCore could not create an anchor. Try again."; return false; }
            if (this == null) { Destroy(result.value.gameObject); return false; }
            _chamber.transform.SetParent(result.value.transform, false);
            _chamber.transform.localPosition = Vector3.zero; _chamber.transform.localRotation = Quaternion.identity;
            if (_anchor != null) Destroy(_anchor.gameObject);
            _anchor = result.value;
            return true;
        }

        private float UiScale => Mathf.Clamp(Screen.dpi / 180f, 1f, 2f);

        private void OnGUI()
        {
            var scale = UiScale;
            var previous = GUI.matrix;
            GUI.matrix = Matrix4x4.Scale(Vector3.one * scale);
            var w = Screen.width / scale; var h = Screen.height / scale;
            GUILayout.BeginArea(new Rect(10, 10, w - 20, 145), GUI.skin.box);
            GUILayout.Label(_status);
            _gateway = GUILayout.TextField(_gateway, 300);
            GUILayout.BeginHorizontal();
            GUI.enabled = !_connection.Busy && !_connection.Connected;
            if (GUILayout.Button("Connect to AIVA")) { PlayerPrefs.SetString("grace-ar-gateway", _gateway); PlayerPrefs.Save(); _connection.Connect(_gateway); }
            GUI.enabled = true;
            if (_connection.Connected && GUILayout.Button("Disconnect")) _connection.Disconnect();
            if (GUILayout.Button("Canvas"))
            {
                if (Uri.TryCreate(_gateway, UriKind.Absolute, out var url) && (url.Scheme == "http" || url.Scheme == "https"))
                    Application.OpenURL(new Uri(url, "/canvas").AbsoluteUri);
            }
            GUILayout.EndHorizontal(); GUILayout.EndArea();
            GUILayout.BeginArea(new Rect(10, h - 250, w - 20, 240), GUI.skin.box);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Place / Move") && !_placementBusy) { _placing = true; _restored = false; _status = "Scan marker first for persistent placement; then tap the floor."; }
            if (GUILayout.Button("Wave")) _chamber.Actions.Wave();
            if (GUILayout.Button("Turn")) _chamber.Actions.TurnAround();
            GUILayout.EndHorizontal();
            _responseScroll = GUILayout.BeginScrollView(_responseScroll, GUILayout.Height(100));
            GUILayout.Label(_connection.Response);
            GUILayout.EndScrollView();
            _prompt = GUILayout.TextField(_prompt, 4000);
            GUI.enabled = _connection.Connected && !_connection.Busy;
            if (GUILayout.Button("Ask Grace")) { _connection.Ask(_prompt); }
            GUI.enabled = true; GUILayout.EndArea(); GUI.matrix = previous;
        }
    }
}
