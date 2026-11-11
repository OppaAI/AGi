#if UNITY_EDITOR
using Aiva.GraceAR;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR.ARSubsystems;
using UnityEditor.XR.ARSubsystems;
using UnityEditor.XR.Management;
using UnityEditor.XR.Management.Metadata;
using UnityEngine.Rendering;
using System.IO;
using System.Linq;

namespace Aiva.GraceAR.Editor
{
    public static class GraceArProjectSetup
    {
        private const string ScenePath = "Assets/GraceAR/GraceAR.unity";

        [MenuItem("Grace AR/Configure Android and Create Scene")]
        public static void Configure()
        {
            PlayerSettings.companyName = "OppaAI";
            PlayerSettings.productName = "Grace AR";
            PlayerSettings.SetApplicationIdentifier(NamedBuildTarget.Android, "ai.oppa.grace.ar");
            PlayerSettings.SetScriptingBackend(NamedBuildTarget.Android, ScriptingImplementation.IL2CPP);
            PlayerSettings.Android.minSdkVersion = AndroidSdkVersions.AndroidApiLevel24;
            PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
            PlayerSettings.defaultInterfaceOrientation = UIOrientation.AutoRotation;
            PlayerSettings.insecureHttpOption = InsecureHttpOption.AlwaysAllowed;
            var playerSettings = new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/ProjectSettings.asset")[0]);
            var input = playerSettings.FindProperty("activeInputHandler");
            if (input != null) { input.intValue = 1; playerSettings.ApplyModifiedProperties(); }

            var xr = XRGeneralSettingsPerBuildTarget.XRGeneralSettingsForBuildTarget(BuildTargetGroup.Android);
            if (xr != null && xr.AssignedSettings != null)
                XRPackageMetadataStore.AssignLoader(xr.AssignedSettings, "UnityEngine.XR.ARCore.ARCoreLoader", BuildTargetGroup.Android);

            // Runtime-created materials otherwise risk having their shaders stripped from Android builds.
            var graphics = new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/GraphicsSettings.asset")[0]);
            var shaders = graphics.FindProperty("m_AlwaysIncludedShaders");
            foreach (var name in new[] { "Standard", "VRM/MToon" })
            {
                var shader = Shader.Find(name);
                if (shader == null) continue;
                var found = Enumerable.Range(0, shaders.arraySize).Any(i => shaders.GetArrayElementAtIndex(i).objectReferenceValue == shader);
                if (!found) { var i = shaders.arraySize++; shaders.GetArrayElementAtIndex(i).objectReferenceValue = shader; }
            }
            graphics.ApplyModifiedProperties();

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var app = new GameObject("Grace AR App");
            var component = app.AddComponent<GraceArApp>();
            var configured = new SerializedObject(component);
            configured.FindProperty("markerLibrary").objectReferenceValue = CreateMarker();
            configured.ApplyModifiedProperties();
            EditorSceneManager.SaveScene(scene, ScenePath);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
            AssetDatabase.SaveAssets();

            Debug.Log(
                "Grace AR scene created. In Project Settings > XR Plug-in Management > Android, " +
                "enable ARCore before building to your phone.");
        }

        private static XRReferenceImageLibrary CreateMarker()
        {
            const string texturePath = "Assets/GraceAR/GraceMarker.png";
            const string libraryPath = "Assets/GraceAR/GraceMarkers.asset";
            if (!File.Exists(texturePath))
            {
                // Deterministic asymmetric texture with plentiful corners for ARCore feature tracking.
                var texture = new Texture2D(600, 600, TextureFormat.RGB24, false);
                var random = new System.Random(734159);
                var cells = new Color[30, 30];
                for (var y = 0; y < 30; y++) for (var x = 0; x < 30; x++)
                    cells[x, y] = random.NextDouble() > 0.5 ? Color.black : Color.white;
                for (var y = 0; y < 600; y++) for (var x = 0; x < 600; x++)
                    texture.SetPixel(x, y, x < 20 || y < 20 || x >= 580 || y >= 580 ? Color.white : cells[x / 20, y / 20]);
                texture.Apply(); File.WriteAllBytes(texturePath, texture.EncodeToPNG()); Object.DestroyImmediate(texture);
                AssetDatabase.ImportAsset(texturePath);
            }
            var library = AssetDatabase.LoadAssetAtPath<XRReferenceImageLibrary>(libraryPath);
            if (library != null) return library;
            library = ScriptableObject.CreateInstance<XRReferenceImageLibrary>();
            library.Add(); library.SetName(0, "GraceRoomMarker");
            library.SetTexture(0, AssetDatabase.LoadAssetAtPath<Texture2D>(texturePath), false);
            library.SetSpecifySize(0, true); library.SetSize(0, new Vector2(0.15f, 0.15f));
            AssetDatabase.CreateAsset(library, libraryPath);
            return library;
        }
    }
}
#endif

