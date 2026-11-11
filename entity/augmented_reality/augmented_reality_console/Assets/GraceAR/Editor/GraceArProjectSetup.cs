#if UNITY_EDITOR
using Aiva.GraceAR;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

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

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var app = new GameObject("Grace AR App");
            app.AddComponent<GraceArApp>();
            EditorSceneManager.SaveScene(scene, ScenePath);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
            AssetDatabase.SaveAssets();

            Debug.Log(
                "Grace AR scene created. In Project Settings > XR Plug-in Management > Android, " +
                "enable ARCore before building to your phone.");
        }
    }
}
#endif

