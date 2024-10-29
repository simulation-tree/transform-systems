using System.Numerics;
using Transforms.Components;

namespace Transforms.Tests
{
    public class LocalToWorldTypeTests
    {
        [Test]
        public void LocalPositionFromWorld()
        {
            LocalToWorld child = new(new Vector3(1, 2, 3), Quaternion.Identity, new Vector3(1, 1, 1));
            LocalToWorld parent = new(new Vector3(5, 0, 0), Quaternion.Identity, new Vector3(2, 3, 2));
            Vector3 localPosition = child.Position;
            Vector3 worldPosition = Vector3.Transform(localPosition, parent.value);
            Assert.That(worldPosition.X, Is.EqualTo(7).Within(0.1f));
            Assert.That(worldPosition.Y, Is.EqualTo(6).Within(0.1f));
            Assert.That(worldPosition.Z, Is.EqualTo(6).Within(0.1f));

            Vector3 desiredWorldPosition = new(1, -2, -2);
            Matrix4x4.Invert(parent.value, out Matrix4x4 invParent);
            Vector3 desiredLocalPosition = Vector3.Transform(desiredWorldPosition, invParent);
            Assert.That(desiredLocalPosition.X, Is.EqualTo(-2f).Within(0.1f));
            Assert.That(desiredLocalPosition.Y, Is.EqualTo(-0.6666f).Within(0.1f));
            Assert.That(desiredLocalPosition.Z, Is.EqualTo(-1f).Within(0.1f));
        }
    }
}
