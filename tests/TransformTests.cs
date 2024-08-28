using System;
using System.Numerics;
using Transforms.Components;
using Transforms.Events;
using Transforms.Systems;
using Unmanaged;

namespace Simulation.Tests
{
    public class TransformTests
    {
        [TearDown]
        public void CleanUp()
        {
            Allocations.ThrowIfAny();
        }

        private static void Simulate(World world)
        {
            world.Submit(new TransformUpdate());
            world.Poll();
        }

        [Test]
        public void CheckAnchorDataType()
        {
            Anchor.value left = new(0.92123f, true);
            Assert.That(left.IsAbsolute, Is.True);
            Assert.That(left.Number, Is.EqualTo(0.92123f).Within(0.1f));

            Anchor.value right = new(456f, false);
            Assert.That(right.IsAbsolute, Is.False);
            Assert.That(right.Number, Is.EqualTo(456f).Within(0.1f));

            Assert.That(left, Is.Not.EqualTo(right));
            Anchor.value up = new(0.5f, true);
            Assert.That(up, Is.Not.EqualTo(left));
            Assert.That(up.IsAbsolute, Is.True);
            Assert.That(up.Number, Is.EqualTo(0.5f).Within(0.1f));

            Anchor.value down = new(-3.1412f, false);
            Assert.That(down, Is.Not.EqualTo(right));
            Assert.That(down.IsAbsolute, Is.False);
            Assert.That(down.Number, Is.EqualTo(-3.1412f).Within(0.1f));

            using RandomGenerator rng = new();
            for (uint i = 0; i < 1024; i++)
            {
                float number = rng.NextFloat(-1000f, 1000f);
                bool isNormalized = rng.NextBool();
                Anchor.value value = new(number, isNormalized);
                value.Number *= 2;
                value.IsAbsolute = !value.IsAbsolute;
                Assert.That(value.Number, Is.EqualTo(number * 2).Within(0.1f));
                Assert.That(value.IsAbsolute, Is.EqualTo(!isNormalized));
            }
        }

        [Test]
        public void AnchorStuff()
        {
            using World world = new();
            using TransformSystem transformSystem = new(world);

            eint entity = world.CreateEntity();
            world.AddComponent(entity, new IsTransform());
            world.AddComponent(entity, new Scale(1920, 1080, 1));

            eint crosshairHorizontal = world.CreateEntity(entity);
            world.AddComponent(crosshairHorizontal, new IsTransform());
            world.AddComponent(crosshairHorizontal, Anchor.Centered);
            world.AddComponent(crosshairHorizontal, new Scale(32, 1, 1));

            eint crosshairVertical = world.CreateEntity(entity);
            world.AddComponent(crosshairVertical, new IsTransform());
            world.AddComponent(crosshairVertical, Anchor.Centered);
            world.AddComponent(crosshairVertical, new Scale(1, 32, 1));

            Simulate(world);

            LocalToWorld horizontalLtw = world.GetComponent<LocalToWorld>(crosshairHorizontal);
            Assert.That(horizontalLtw.Position.X, Is.EqualTo(960));
            Assert.That(horizontalLtw.Position.Y, Is.EqualTo(540));
            Assert.That(horizontalLtw.Scale.X, Is.EqualTo(32));
            Assert.That(horizontalLtw.Scale.Y, Is.EqualTo(1));

            LocalToWorld verticalLtw = world.GetComponent<LocalToWorld>(crosshairVertical);
            Assert.That(verticalLtw.Position.X, Is.EqualTo(960));
            Assert.That(verticalLtw.Position.Y, Is.EqualTo(540));
            Assert.That(verticalLtw.Scale.X, Is.EqualTo(1));
            Assert.That(verticalLtw.Scale.Y, Is.EqualTo(32));
        }

        [Test]
        public void DeepAnchoring()
        {
            using World world = new();
            using TransformSystem transformSystem = new(world);

            eint entity = world.CreateEntity();
            world.AddComponent(entity, new IsTransform());
            world.AddComponent(entity, new Scale(1920, 1080, 1));

            eint bottomLeftCanvas = world.CreateEntity(entity);
            world.AddComponent(bottomLeftCanvas, new IsTransform());
            world.AddComponent(bottomLeftCanvas, new Anchor(new(0f, false), new(0f, false), new(0f, false), new(0.5f, false), new(0.5f, false), new(1f, false)));

            eint pointInsideCanvas = world.CreateEntity(bottomLeftCanvas);
            world.AddComponent(pointInsideCanvas, new IsTransform());
            world.AddComponent(pointInsideCanvas, new Position(32f, 32f, 0));
            world.AddComponent(pointInsideCanvas, new Scale(32, 32, 1));
            world.AddComponent(pointInsideCanvas, Anchor.Centered);

            eint copyPoint = world.CreateEntity(pointInsideCanvas);
            world.AddComponent(copyPoint, new IsTransform());
            world.AddComponent(copyPoint, new Anchor(new(0f, false), new(0f, false), new(0f, false), new(1f, false), new(1f, false), new(1f, false)));

            Simulate(world);

            LocalToWorld bottomLeftCanvasLtw = world.GetComponent<LocalToWorld>(bottomLeftCanvas);
            Assert.That(bottomLeftCanvasLtw.Position.X, Is.EqualTo(0));
            Assert.That(bottomLeftCanvasLtw.Position.Y, Is.EqualTo(0));
            Assert.That(bottomLeftCanvasLtw.Scale.X, Is.EqualTo(960));
            Assert.That(bottomLeftCanvasLtw.Scale.Y, Is.EqualTo(540));

            LocalToWorld pointInsideCanvasLtw = world.GetComponent<LocalToWorld>(pointInsideCanvas);
            Assert.That(pointInsideCanvasLtw.Position.X, Is.EqualTo(1920 / 4 + 32));
            Assert.That(pointInsideCanvasLtw.Position.Y, Is.EqualTo(1080 / 4 + 32));
            Assert.That(pointInsideCanvasLtw.Scale.X, Is.EqualTo(32));
            Assert.That(pointInsideCanvasLtw.Scale.Y, Is.EqualTo(32));

            LocalToWorld copyPointLtw = world.GetComponent<LocalToWorld>(copyPoint);
            Assert.That(copyPointLtw.Position.X, Is.EqualTo(pointInsideCanvasLtw.Position.X));
            Assert.That(copyPointLtw.Position.Y, Is.EqualTo(pointInsideCanvasLtw.Position.Y));
            Assert.That(copyPointLtw.Scale.X, Is.EqualTo(pointInsideCanvasLtw.Scale.X));
            Assert.That(copyPointLtw.Scale.Y, Is.EqualTo(pointInsideCanvasLtw.Scale.Y));
        }

        [Test]
        public void CreateLocalToWorldFromComponents()
        {
            using World world = new();
            using TransformSystem transformSystem = new(world);

            eint entity = world.CreateEntity();
            world.AddComponent(entity, new Position(2, 2, 6));
            world.AddComponent(entity, new Scale(2, 1, 2));
            world.AddComponent(entity, new IsTransform());

            Simulate(world);

            Assert.That(world.ContainsComponent<LocalToWorld>(entity), Is.True);
            LocalToWorld ltw = world.GetComponent<LocalToWorld>(entity);
            Assert.That(ltw.value, Is.EqualTo(Matrix4x4.CreateScale(2, 1, 2) * Matrix4x4.CreateTranslation(2, 2, 6)));
        }

        [Test]
        public void ScaleChildRelativeToParent()
        {
            using World world = new();
            using TransformSystem transformSystem = new(world);

            eint parent = world.CreateEntity();
            world.AddComponent(parent, new Position(5, 0, 0));
            world.AddComponent(parent, EulerAngles.CreateFromDegrees(0f, 90f, 0f));
            world.AddComponent(parent, new Scale(2, 2, 2));
            world.AddComponent(parent, new IsTransform());

            eint child = world.CreateEntity();
            world.AddComponent(child, new Position(0, 0, 2));
            world.AddComponent(child, new IsTransform());
            world.SetParent(child, parent);

            Simulate(world);

            Assert.That(world.ContainsComponent<LocalToWorld>(child), Is.True);
            LocalToWorld childLtw = world.GetComponent<LocalToWorld>(child);
            Assert.That(childLtw.Position.X, Is.EqualTo(9).Within(0.1f));
            Assert.That(childLtw.Position.Y, Is.EqualTo(0f).Within(0.1f));
            Assert.That(childLtw.Position.Z, Is.EqualTo(0f).Within(0.1f));
            Assert.That(childLtw.Scale.X, Is.EqualTo(2).Within(0.1f));
            Assert.That(childLtw.Scale.Y, Is.EqualTo(2).Within(0.1f));
            Assert.That(childLtw.Scale.Z, Is.EqualTo(2).Within(0.1f));
        }

        [Test]
        public void WorldRotationFromLocal()
        {
            Quaternion sampleRotation = Quaternion.CreateFromYawPitchRoll(0.5f, 0.5f, 0.5f);
            using World world = new();
            using TransformSystem transforms = new(world);

            eint entity = world.CreateEntity();
            world.AddComponent(entity, new IsTransform());
            world.AddComponent(entity, new Rotation(sampleRotation));

            Simulate(world);

            Quaternion worldRotation = world.GetComponent<WorldRotation>(entity).value;
            Assert.That(worldRotation, Is.EqualTo(sampleRotation));
        }

        [Test]
        public void WorldRotationFromNestedChild()
        {
            Quaternion localRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.25f);
            using World world = new();
            using TransformSystem transforms = new(world);

            eint parent = world.CreateEntity();
            world.AddComponent(parent, new IsTransform());
            world.AddComponent(parent, new Rotation(localRotation));

            eint child = world.CreateEntity(parent);
            world.AddComponent(child, new IsTransform());
            world.AddComponent(child, new Rotation(localRotation));

            Simulate(world);

            Quaternion worldRotation = world.GetComponent<WorldRotation>(child).value;
            Quaternion expectedWorldRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f);
            Assert.That(worldRotation.X, Is.EqualTo(expectedWorldRotation.X).Within(0.01f));
            Assert.That(worldRotation.Y, Is.EqualTo(expectedWorldRotation.Y).Within(0.01f));
            Assert.That(worldRotation.Z, Is.EqualTo(expectedWorldRotation.Z).Within(0.01f));

            Matrix4x4.Invert(world.GetComponent<LocalToWorld>(parent).value, out Matrix4x4 wtl);
            Quaternion localRotationAgain = Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(wtl) * worldRotation);
            Assert.That(localRotationAgain.X, Is.EqualTo(localRotation.X).Within(0.01f));
            Assert.That(localRotationAgain.Y, Is.EqualTo(localRotation.Y).Within(0.01f));
            Assert.That(localRotationAgain.Z, Is.EqualTo(localRotation.Z).Within(0.01f));
        }

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
