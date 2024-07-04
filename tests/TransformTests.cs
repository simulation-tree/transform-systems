using Game.Systems;
using System.Numerics;
using Transforms.Components;
using Transforms.Events;
using Unmanaged;

namespace Game.Tests
{
    public class TransformTests
    {
        [TearDown]
        public void CleanUp()
        {
            Allocations.ThrowIfAny();
        }

        [Test]
        public void CreateLocalToWorldFromComponents()
        {
            using World world = new();
            using TransformSystem transformSystem = new(world);

            EntityID entity = world.CreateEntity();
            world.AddComponent(entity, new Position(2, 2, 6));
            world.AddComponent(entity, new Scale(2, 1, 2));
            world.AddComponent(entity, new IsTransform());

            world.Submit(new TransformUpdate());
            world.Poll();

            Assert.That(world.ContainsComponent<LocalToWorld>(entity), Is.True);
            LocalToWorld ltw = world.GetComponent<LocalToWorld>(entity);
            Assert.That(ltw.value, Is.EqualTo(Matrix4x4.CreateScale(2, 1, 2) * Matrix4x4.CreateTranslation(2, 2, 6)));
        }

        [Test]
        public void ScaleChildRelativeToParent()
        {
            using World world = new();
            using TransformSystem transformSystem = new(world);

            EntityID parent = world.CreateEntity();
            world.AddComponent(parent, new Position(5, 0, 0));
            world.AddComponent(parent, EulerAngles.CreateFromDegrees(0f, 90f, 0f));
            world.AddComponent(parent, new Scale(2, 2, 2));
            world.AddComponent(parent, new IsTransform());

            EntityID child = world.CreateEntity();
            world.AddComponent(child, new Position(0, 0, 2));
            world.AddComponent(child, new IsTransform());
            world.SetParent(child, parent);

            world.Submit(new TransformUpdate());
            world.Poll();

            Assert.That(world.ContainsComponent<LocalToWorld>(child), Is.True);
            LocalToWorld ltw = world.GetComponent<LocalToWorld>(child);
            Assert.That(ltw.Position.X, Is.InRange(9 - 0.02f, 9 + 0.02f));
            Assert.That(ltw.Position.Y, Is.InRange(-0.02f, 0.02f));
            Assert.That(ltw.Position.Z, Is.InRange(-0.02f, 0.02f));
            Assert.That(ltw.Scale.X, Is.InRange(2 - 0.02f, 2 + 0.02f));
            Assert.That(ltw.Scale.Y, Is.InRange(2 - 0.02f, 2 + 0.02f));
            Assert.That(ltw.Scale.Z, Is.InRange(2 - 0.02f, 2 + 0.02f));
        }
    }
}
