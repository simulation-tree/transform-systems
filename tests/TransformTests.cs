using Simulation.Tests;
using System;
using System.Numerics;
using Transforms.Components;
using Transforms.Systems;
using Worlds;

namespace Transforms.Tests
{
    public class TransformTests : SimulationTests
    {
        protected override void SetUp()
        {
            base.SetUp();
            ComponentType.Register<IsTransform>();
            ComponentType.Register<Position>();
            ComponentType.Register<Rotation>();
            ComponentType.Register<WorldRotation>();
            ComponentType.Register<EulerAngles>();
            ComponentType.Register<Scale>();
            ComponentType.Register<Anchor>();
            ComponentType.Register<Pivot>();
            ComponentType.Register<LocalToWorld>();
            Simulator.AddSystem(new TransformSystem());
        }

        [Test]
        public void AnchorStuff()
        {
            uint entity = World.CreateEntity();
            World.AddComponent(entity, new IsTransform());
            World.AddComponent(entity, new Scale(1920, 1080, 1));

            uint crosshairHorizontal = World.CreateEntity();
            World.SetParent(crosshairHorizontal, entity);
            World.AddComponent(crosshairHorizontal, new IsTransform());
            World.AddComponent(crosshairHorizontal, Anchor.Centered);
            World.AddComponent(crosshairHorizontal, new Scale(32, 1, 1));

            uint crosshairVertical = World.CreateEntity();
            World.SetParent(crosshairVertical, entity);
            World.AddComponent(crosshairVertical, new IsTransform());
            World.AddComponent(crosshairVertical, Anchor.Centered);
            World.AddComponent(crosshairVertical, new Scale(1, 32, 1));

            Simulator.Update(TimeSpan.FromSeconds(0.01f));

            LocalToWorld horizontalLtw = World.GetComponent<LocalToWorld>(crosshairHorizontal);
            Assert.That(horizontalLtw.Position.X, Is.EqualTo(960));
            Assert.That(horizontalLtw.Position.Y, Is.EqualTo(540));
            Assert.That(horizontalLtw.Scale.X, Is.EqualTo(32));
            Assert.That(horizontalLtw.Scale.Y, Is.EqualTo(1));

            LocalToWorld verticalLtw = World.GetComponent<LocalToWorld>(crosshairVertical);
            Assert.That(verticalLtw.Position.X, Is.EqualTo(960));
            Assert.That(verticalLtw.Position.Y, Is.EqualTo(540));
            Assert.That(verticalLtw.Scale.X, Is.EqualTo(1));
            Assert.That(verticalLtw.Scale.Y, Is.EqualTo(32));
        }

        [Test]
        public void DeepAnchoring()
        {
            uint entity = World.CreateEntity();
            World.AddComponent(entity, new IsTransform());
            World.AddComponent(entity, new Scale(1920, 1080, 1));

            Anchor anchor = new(new(0f, false), new(0f, false), new(0f, false), new(0.5f, false), new(0.5f, false), new(1f, false));
            uint bottomLeftCanvas = World.CreateEntity();
            World.SetParent(bottomLeftCanvas, entity);
            World.AddComponent(bottomLeftCanvas, new IsTransform());
            World.AddComponent(bottomLeftCanvas, anchor);

            uint pointInsideCanvas = World.CreateEntity();
            World.SetParent(pointInsideCanvas, bottomLeftCanvas);
            World.AddComponent(pointInsideCanvas, new IsTransform());
            World.AddComponent(pointInsideCanvas, new Position(32f, 32f, 0));
            World.AddComponent(pointInsideCanvas, new Scale(32, 32, 1));
            World.AddComponent(pointInsideCanvas, Anchor.Centered);

            uint copyPoint = World.CreateEntity();
            World.SetParent(copyPoint, pointInsideCanvas);
            World.AddComponent(copyPoint, new IsTransform());
            World.AddComponent(copyPoint, new Anchor(new(0f, false), new(0f, false), new(0f, false), new(1f, false), new(1f, false), new(1f, false)));

            Simulator.Update(TimeSpan.FromSeconds(0.01f));

            LocalToWorld bottomLeftCanvasLtw = World.GetComponent<LocalToWorld>(bottomLeftCanvas);
            Assert.That(bottomLeftCanvasLtw.Position.X, Is.EqualTo(0));
            Assert.That(bottomLeftCanvasLtw.Position.Y, Is.EqualTo(0));
            Assert.That(bottomLeftCanvasLtw.Scale.X, Is.EqualTo(960));
            Assert.That(bottomLeftCanvasLtw.Scale.Y, Is.EqualTo(540));

            LocalToWorld pointInsideCanvasLtw = World.GetComponent<LocalToWorld>(pointInsideCanvas);
            Assert.That(pointInsideCanvasLtw.Position.X, Is.EqualTo(1920 / 4 + 32));
            Assert.That(pointInsideCanvasLtw.Position.Y, Is.EqualTo(1080 / 4 + 32));
            Assert.That(pointInsideCanvasLtw.Scale.X, Is.EqualTo(32));
            Assert.That(pointInsideCanvasLtw.Scale.Y, Is.EqualTo(32));

            LocalToWorld copyPointLtw = World.GetComponent<LocalToWorld>(copyPoint);
            Assert.That(copyPointLtw.Position.X, Is.EqualTo(pointInsideCanvasLtw.Position.X));
            Assert.That(copyPointLtw.Position.Y, Is.EqualTo(pointInsideCanvasLtw.Position.Y));
            Assert.That(copyPointLtw.Scale.X, Is.EqualTo(pointInsideCanvasLtw.Scale.X));
            Assert.That(copyPointLtw.Scale.Y, Is.EqualTo(pointInsideCanvasLtw.Scale.Y));
        }

        [Test]
        public void AnchorWithBorder()
        {
            uint canvas = World.CreateEntity();
            World.AddComponent(canvas, new IsTransform());
            World.AddComponent(canvas, new Scale(1920, 1080, 1));

            uint bordered = World.CreateEntity();
            World.SetParent(bordered, canvas);
            World.AddComponent(bordered, new IsTransform());
            World.AddComponent(bordered, new Scale(1, 1));
            World.AddComponent(bordered, new Anchor(new(10f, true), new(10f, true), default, new(10f, true), new(10f, true), default));

            Simulator.Update(TimeSpan.FromSeconds(0.01f));

            LocalToWorld borderedLtw = World.GetComponent<LocalToWorld>(bordered);
            Assert.That(borderedLtw.Position.X, Is.EqualTo(10));
            Assert.That(borderedLtw.Position.Y, Is.EqualTo(10));
            Assert.That(borderedLtw.Scale.X, Is.EqualTo(1900));
            Assert.That(borderedLtw.Scale.Y, Is.EqualTo(1060));
        }

        [Test]
        public void CreateLocalToWorldFromComponents()
        {
            uint entity = World.CreateEntity();
            World.AddComponent(entity, new Position(2, 2, 6));
            World.AddComponent(entity, new Scale(2, 1, 2));
            World.AddComponent(entity, new IsTransform());

            Simulator.Update(TimeSpan.FromSeconds(0.01f));

            Assert.That(World.ContainsComponent<LocalToWorld>(entity), Is.True);
            LocalToWorld ltw = World.GetComponent<LocalToWorld>(entity);
            Assert.That(ltw.value, Is.EqualTo(Matrix4x4.CreateScale(2, 1, 2) * Matrix4x4.CreateTranslation(2, 2, 6)));
        }

        [Test]
        public void ScaleChildRelativeToParent()
        {
            uint parent = World.CreateEntity();
            World.AddComponent(parent, new Position(5, 0, 0));
            World.AddComponent(parent, EulerAngles.CreateFromDegrees(0f, 90f, 0f));
            World.AddComponent(parent, new Scale(2, 2, 2));
            World.AddComponent(parent, new IsTransform());

            uint child = World.CreateEntity();
            World.SetParent(child, parent);
            World.AddComponent(child, new Position(0, 0, 2));
            World.AddComponent(child, new IsTransform());

            Assert.That(World.GetParent(child), Is.EqualTo(parent));
            Simulator.Update(TimeSpan.FromSeconds(0.01f));

            Assert.That(World.ContainsComponent<LocalToWorld>(child), Is.True);
            LocalToWorld childLtw = World.GetComponent<LocalToWorld>(child);
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
            uint entity = World.CreateEntity();
            World.AddComponent(entity, new IsTransform());
            World.AddComponent(entity, new Rotation(sampleRotation));

            Simulator.Update(TimeSpan.FromSeconds(0.01f));

            Quaternion worldRotation = World.GetComponent<WorldRotation>(entity).value;
            Assert.That(worldRotation, Is.EqualTo(sampleRotation));
        }

        [Test]
        public void WorldRotationFromNestedChild()
        {
            Quaternion localRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.25f);

            uint parent = World.CreateEntity();
            World.AddComponent(parent, new IsTransform());
            World.AddComponent(parent, new Rotation(localRotation));

            uint child = World.CreateEntity();
            World.SetParent(child, parent);
            World.AddComponent(child, new IsTransform());
            World.AddComponent(child, new Rotation(localRotation));

            Simulator.Update(TimeSpan.FromSeconds(0.01f));

            Quaternion worldRotation = World.GetComponent<WorldRotation>(child).value;
            Quaternion expectedWorldRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f);
            Assert.That(worldRotation.X, Is.EqualTo(expectedWorldRotation.X).Within(0.01f));
            Assert.That(worldRotation.Y, Is.EqualTo(expectedWorldRotation.Y).Within(0.01f));
            Assert.That(worldRotation.Z, Is.EqualTo(expectedWorldRotation.Z).Within(0.01f));

            Matrix4x4.Invert(World.GetComponent<LocalToWorld>(parent).value, out Matrix4x4 wtl);
            Quaternion localRotationAgain = Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(wtl) * worldRotation);
            Assert.That(localRotationAgain.X, Is.EqualTo(localRotation.X).Within(0.01f));
            Assert.That(localRotationAgain.Y, Is.EqualTo(localRotation.Y).Within(0.01f));
            Assert.That(localRotationAgain.Z, Is.EqualTo(localRotation.Z).Within(0.01f));
        }
    }
}
