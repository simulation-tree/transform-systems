using Simulation.Tests;
using Transforms.Messages;
using Transforms.Systems;
using Types;
using Worlds;

namespace Transforms.Tests
{
    public abstract class TransformSystemsTests : SimulationTests
    {
        public World world;

        static TransformSystemsTests()
        {
            MetadataRegistry.Load<TransformsMetadataBank>();
        }

        protected override void SetUp()
        {
            base.SetUp();
            Schema schema = new();
            schema.Load<TransformsSchemaBank>();
            world = new(schema);
            Simulator.Add(new TransformSystem(Simulator, world));
        }

        protected override void TearDown()
        {
            Simulator.Remove<TransformSystem>();
            world.Dispose();
            base.TearDown();
        }

        protected override void Update(double deltaTime)
        {
            Simulator.Broadcast(new TransformUpdate());
        }
    }
}