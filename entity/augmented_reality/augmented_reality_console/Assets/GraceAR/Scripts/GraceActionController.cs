using System.Collections;
using UnityEngine;

namespace Aiva.GraceAR
{
    public sealed class GraceActionController : MonoBehaviour
    {
        private Transform _model;
        private Animator _animator;
        private Transform _rightUpperArm;
        private Transform _rightLowerArm;
        private Quaternion _upperRest;
        private Quaternion _lowerRest;
        private Vector3 _modelRestPosition;
        private Coroutine _action;

        public void Bind(GameObject model)
        {
            _model = model.transform;
            _modelRestPosition = _model.localPosition;
            _animator = model.GetComponent<Animator>();
            if (_animator == null || !_animator.isHuman)
            {
                return;
            }

            _rightUpperArm = _animator.GetBoneTransform(HumanBodyBones.RightUpperArm);
            _rightLowerArm = _animator.GetBoneTransform(HumanBodyBones.RightLowerArm);
            if (_rightUpperArm != null) _upperRest = _rightUpperArm.localRotation;
            if (_rightLowerArm != null) _lowerRest = _rightLowerArm.localRotation;
        }

        private void LateUpdate()
        {
            if (_model == null) return;
            var bob = Mathf.Sin(Time.time * 0.8f) * 0.012f;
            _model.localPosition = _modelRestPosition + Vector3.up * bob;
        }

        public void Wave()
        {
            if (_rightUpperArm == null || _rightLowerArm == null) return;
            if (_action != null) StopCoroutine(_action);
            _action = StartCoroutine(WaveRoutine());
        }

        public void TurnAround()
        {
            if (_model == null) return;
            if (_action != null) StopCoroutine(_action);
            _action = StartCoroutine(TurnRoutine());
        }

        private IEnumerator WaveRoutine()
        {
            var raisedUpper = _upperRest * Quaternion.Euler(-15f, 10f, -105f);
            var raisedLower = _lowerRest * Quaternion.Euler(0f, -15f, -45f);
            yield return RotateBones(_upperRest, raisedUpper, _lowerRest, raisedLower, 0.45f);

            for (var wave = 0; wave < 3; wave++)
            {
                var a = raisedLower * Quaternion.Euler(0f, 0f, -18f);
                var b = raisedLower * Quaternion.Euler(0f, 0f, 18f);
                yield return RotateBones(raisedUpper, raisedUpper, a, b, 0.18f);
                yield return RotateBones(raisedUpper, raisedUpper, b, a, 0.18f);
            }

            yield return RotateBones(raisedUpper, _upperRest, raisedLower, _lowerRest, 0.45f);
            _action = null;
        }

        private IEnumerator RotateBones(
            Quaternion upperFrom,
            Quaternion upperTo,
            Quaternion lowerFrom,
            Quaternion lowerTo,
            float duration)
        {
            var elapsed = 0f;
            while (elapsed < duration)
            {
                elapsed += Time.deltaTime;
                var t = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(elapsed / duration));
                _rightUpperArm.localRotation = Quaternion.Slerp(upperFrom, upperTo, t);
                _rightLowerArm.localRotation = Quaternion.Slerp(lowerFrom, lowerTo, t);
                yield return null;
            }
        }

        private IEnumerator TurnRoutine()
        {
            var start = _model.localRotation;
            var elapsed = 0f;
            while (elapsed < 1.4f)
            {
                elapsed += Time.deltaTime;
                var angle = Mathf.Clamp01(elapsed / 1.4f) * 360f;
                _model.localRotation = start * Quaternion.AngleAxis(angle, Vector3.up);
                yield return null;
            }
            _model.localRotation = start;
            _action = null;
        }
    }
}

