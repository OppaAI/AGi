using UnityEngine;

namespace Aiva.GraceAR
{
    public sealed class GraceChamber : MonoBehaviour
    {
        public Transform ModelMount { get; private set; }
        public GraceActionController Actions { get; private set; }

        public static GraceChamber Create()
        {
            var root = new GameObject("Grace Chamber");
            var chamber = root.AddComponent<GraceChamber>();
            chamber.Build();
            return chamber;
        }

        private void Build()
        {
            var metal = CreateMaterial(new Color(0.10f, 0.12f, 0.15f), 1f, 0.8f, 0.7f);
            var glass = CreateTransparentMaterial(new Color(0.45f, 0.95f, 1f, 0.16f));
            var liquid = CreateTransparentMaterial(new Color(0.05f, 0.65f, 0.72f, 0.10f));
            var cyan = CreateMaterial(new Color(0.1f, 1f, 0.92f), 1f, 0.2f, 0.9f);

            CreateCylinder("Glass", transform, new Vector3(0f, 1.1f, 0f), new Vector3(0.66f, 1.1f, 0.66f), glass);
            CreateCylinder("Liquid", transform, new Vector3(0f, 1.1f, 0f), new Vector3(0.62f, 1.04f, 0.62f), liquid);
            CreateCylinder("Base", transform, new Vector3(0f, 0.08f, 0f), new Vector3(0.78f, 0.08f, 0.78f), metal);
            CreateCylinder("Top", transform, new Vector3(0f, 2.12f, 0f), new Vector3(0.78f, 0.08f, 0.78f), metal);
            CreateCylinder("Base Glow", transform, new Vector3(0f, 0.18f, 0f), new Vector3(0.70f, 0.018f, 0.70f), cyan);
            CreateCylinder("Top Glow", transform, new Vector3(0f, 2.02f, 0f), new Vector3(0.70f, 0.018f, 0.70f), cyan);

            ModelMount = new GameObject("Grace Model Mount").transform;
            ModelMount.SetParent(transform, false);
            ModelMount.localPosition = new Vector3(0f, 0.22f, 0f);
            ModelMount.localScale = Vector3.one * 0.92f;

            Actions = gameObject.AddComponent<GraceActionController>();
            AddBubbles();
        }

        private void AddBubbles()
        {
            var bubblesObject = new GameObject("Liquid Bubbles");
            bubblesObject.transform.SetParent(transform, false);
            bubblesObject.transform.localPosition = new Vector3(0f, 0.22f, 0f);
            var particles = bubblesObject.AddComponent<ParticleSystem>();
            var main = particles.main;
            main.startLifetime = 4.5f;
            main.startSpeed = 0.28f;
            main.startSize = 0.018f;
            main.startColor = new Color(0.65f, 0.98f, 1f, 0.55f);
            main.simulationSpace = ParticleSystemSimulationSpace.Local;
            main.maxParticles = 45;
            var emission = particles.emission;
            emission.rateOverTime = 5f;
            var shape = particles.shape;
            shape.shapeType = ParticleSystemShapeType.Cylinder;
            shape.radius = 0.48f;
            shape.height = 0.1f;
            shape.position = Vector3.zero;
        }

        private static void CreateCylinder(string name, Transform parent, Vector3 position, Vector3 scale, Material material)
        {
            var item = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            item.name = name;
            item.transform.SetParent(parent, false);
            item.transform.localPosition = position;
            item.transform.localScale = scale;
            item.GetComponent<Renderer>().sharedMaterial = material;
            Destroy(item.GetComponent<Collider>());
        }

        private static Material CreateMaterial(Color color, float alpha, float metallic, float smoothness)
        {
            var shader = Shader.Find("Standard");
            var material = new Material(shader) { color = new Color(color.r, color.g, color.b, alpha) };
            material.SetFloat("_Metallic", metallic);
            material.SetFloat("_Glossiness", smoothness);
            return material;
        }

        private static Material CreateTransparentMaterial(Color color)
        {
            var material = CreateMaterial(color, color.a, 0f, 0.8f);
            material.SetFloat("_Mode", 3f);
            material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            material.SetInt("_ZWrite", 0);
            material.DisableKeyword("_ALPHATEST_ON");
            material.EnableKeyword("_ALPHABLEND_ON");
            material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            material.renderQueue = 3000;
            return material;
        }
    }
}

