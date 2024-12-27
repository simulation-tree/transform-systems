using Simulation.Components;
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
        static TransformTests()
        {
            TypeLayout.Register<IsTransform>("IsTransform");
            TypeLayout.Register<Position>("Position");
            TypeLayout.Register<Rotation>("Rotation");
            TypeLayout.Register<EulerAngles>("EulerAngles");
            TypeLayout.Register<WorldRotation>("WorldRotation");
            TypeLayout.Register<Scale>("Scale");
            TypeLayout.Register<Anchor>("Anchor");
            TypeLayout.Register<Pivot>("Pivot");
            TypeLayout.Register<LocalToWorld>("LocalToWorld");
        }

        protected override void SetUp()
        {
            base.SetUp();
            world.Schema.RegisterComponent<IsTransform>();
            world.Schema.RegisterComponent<Position>();
            world.Schema.RegisterComponent<Rotation>();
            world.Schema.RegisterComponent<EulerAngles>();
            world.Schema.RegisterComponent<WorldRotation>();
            world.Schema.RegisterComponent<Scale>();
            world.Schema.RegisterComponent<Anchor>();
            world.Schema.RegisterComponent<Pivot>();
            world.Schema.RegisterComponent<LocalToWorld>();
            simulator.AddSystem<TransformSystem>();
        }

        [Test]
        public void AnchorStuff()
        {
            uint entity = world.CreateEntity();
            world.AddComponent(entity, new IsTransform());
            world.AddComponent(entity, new Scale(1920, 1080, 1));

            uint crosshairHorizontal = world.CreateEntity();
            world.SetParent(crosshairHorizontal, entity);
            world.AddComponent(crosshairHorizontal, new IsTransform());
            world.AddComponent(crosshairHorizontal, Anchor.Centered);
            world.AddComponent(crosshairHorizontal, new Scale(32, 1, 1));

            uint crosshairVertical = world.CreateEntity();
            world.SetParent(crosshairVertical, entity);
            world.AddComponent(crosshairVertical, new IsTransform());
            world.AddComponent(crosshairVertical, Anchor.Centered);
            world.AddComponent(crosshairVertical, new Scale(1, 32, 1));

            simulator.Update(TimeSpan.FromSeconds(0.01f));

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
            uint entity = world.CreateEntity();
            world.AddComponent(entity, new IsTransform());
            world.AddComponent(entity, new Scale(1920, 1080, 1));

            Anchor anchor = new(new(0f, false), new(0f, false), new(0f, false), new(0.5f, false), new(0.5f, false), new(1f, false));
            uint bottomLeftCanvas = world.CreateEntity();
            world.SetParent(bottomLeftCanvas, entity);
            world.AddComponent(bottomLeftCanvas, new IsTransform());
            world.AddComponent(bottomLeftCanvas, anchor);

            uint pointInsideCanvas = world.CreateEntity();
            world.SetParent(pointInsideCanvas, bottomLeftCanvas);
            world.AddComponent(pointInsideCanvas, new IsTransform());
            world.AddComponent(pointInsideCanvas, new Position(32f, 32f, 0));
            world.AddComponent(pointInsideCanvas, new Scale(32, 32, 1));
            world.AddComponent(pointInsideCanvas, Anchor.Centered);

            uint copyPoint = world.CreateEntity();
            world.SetParent(copyPoint, pointInsideCanvas);
            world.AddComponent(copyPoint, new IsTransform());
            world.AddComponent(copyPoint, new Anchor(new(0f, false), new(0f, false), new(0f, false), new(1f, false), new(1f, false), new(1f, false)));

            simulator.Update(TimeSpan.FromSeconds(0.01f));

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
        public void AnchorWithBorder()
        {
            uint canvas = world.CreateEntity();
            world.AddComponent(canvas, new IsTransform());
            world.AddComponent(canvas, new Scale(1920, 1080, 1));

            uint bordered = world.CreateEntity();
            world.SetParent(bordered, canvas);
            world.AddComponent(bordered, new IsTransform());
            world.AddComponent(bordered, new Scale(1, 1));
            world.AddComponent(bordered, new Anchor(new(10f, true), new(10f, true), default, new(10f, true), new(10f, true), default));

            simulator.Update(TimeSpan.FromSeconds(0.01f));

            LocalToWorld borderedLtw = world.GetComponent<LocalToWorld>(bordered);
            Assert.That(borderedLtw.Position.X, Is.EqualTo(10));
            Assert.That(borderedLtw.Position.Y, Is.EqualTo(10));
            Assert.That(borderedLtw.Scale.X, Is.EqualTo(1900));
            Assert.That(borderedLtw.Scale.Y, Is.EqualTo(1060));
        }

        [Test]
        public void CreateLocalToWorldFromComponents()
        {
            uint entity = world.CreateEntity();
            world.AddComponent(entity, new Position(2, 2, 6));
            world.AddComponent(entity, new Scale(2, 1, 2));
            world.AddComponent(entity, new IsTransform());

            simulator.Update(TimeSpan.FromSeconds(0.01f));

            Assert.That(world.ContainsComponent<LocalToWorld>(entity), Is.True);
            LocalToWorld ltw = world.GetComponent<LocalToWorld>(entity);
            Assert.That(ltw.value, Is.EqualTo(Matrix4x4.CreateScale(2, 1, 2) * Matrix4x4.CreateTranslation(2, 2, 6)));
        }

        [Test]
        public void ScaleChildRelativeToParent()
        {
            uint parent = world.CreateEntity();
            world.AddComponent(parent, new Position(5, 0, 0));
            world.AddComponent(parent, EulerAngles.CreateFromDegrees(0f, 90f, 0f));
            world.AddComponent(parent, new Scale(2, 2, 2));
            world.AddComponent(parent, new IsTransform());

            uint child = world.CreateEntity();
            world.SetParent(child, parent);
            world.AddComponent(child, new Position(0, 0, 2));
            world.AddComponent(child, new IsTransform());

            Assert.That(world.GetParent(child), Is.EqualTo(parent));
            simulator.Update(TimeSpan.FromSeconds(0.01f));

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
            uint entity = world.CreateEntity();
            world.AddComponent(entity, new IsTransform());
            world.AddComponent(entity, new Rotation(sampleRotation));

            simulator.Update(TimeSpan.FromSeconds(0.01f));

            Quaternion worldRotation = world.GetComponent<WorldRotation>(entity).value;
            Assert.That(worldRotation, Is.EqualTo(sampleRotation));
        }

        [Test]
        public void WorldRotationFromNestedChild()
        {
            Quaternion localRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.25f);

            uint parent = world.CreateEntity();
            world.AddComponent(parent, new IsTransform());
            world.AddComponent(parent, new Rotation(localRotation));

            uint child = world.CreateEntity();
            world.SetParent(child, parent);
            world.AddComponent(child, new IsTransform());
            world.AddComponent(child, new Rotation(localRotation));

            simulator.Update(TimeSpan.FromSeconds(0.01f));

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
    }
}
