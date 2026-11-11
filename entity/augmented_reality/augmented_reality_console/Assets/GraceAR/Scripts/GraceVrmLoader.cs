using System;
using System.Collections;
using System.IO;
using System.Threading.Tasks;
using UniGLTF;
using UnityEngine;
using UnityEngine.Networking;
using VRM;

namespace Aiva.GraceAR
{
    public sealed class GraceVrmLoader : MonoBehaviour
    {
        private RuntimeGltfInstance _instance;

        public event Action<GameObject> Loaded;
        public event Action<string> Failed;

        public void LoadInto(Transform parent)
        {
            StartCoroutine(DownloadAndLoad(parent));
        }

        private IEnumerator DownloadAndLoad(Transform parent)
        {
            var path = Path.Combine(Application.streamingAssetsPath, "Grace.vrm");
            using var request = UnityWebRequest.Get(path);
            yield return request.SendWebRequest();

            if (request.result != UnityWebRequest.Result.Success)
            {
                Failed?.Invoke($"Could not load Grace.vrm: {request.error}");
                yield break;
            }

            var task = LoadBytes(parent, request.downloadHandler.data);
            while (!task.IsCompleted)
            {
                yield return null;
            }

            if (task.IsFaulted)
            {
                Failed?.Invoke(task.Exception?.GetBaseException().Message ?? "Unknown VRM loading error");
            }
        }

        private async Task LoadBytes(Transform parent, byte[] bytes)
        {
            _instance = await VrmUtility.LoadBytesAsync(
                "Grace.vrm",
                bytes,
                new RuntimeOnlyAwaitCaller());

            _instance.transform.SetParent(parent, false);
            _instance.transform.localPosition = Vector3.zero;
            _instance.transform.localRotation = Quaternion.identity;
            _instance.transform.localScale = Vector3.one;
            _instance.EnableUpdateWhenOffscreen();
            _instance.ShowMeshes();
            Loaded?.Invoke(_instance.gameObject);
        }

        private void OnDestroy()
        {
            _instance?.Dispose();
            _instance = null;
        }
    }
}

