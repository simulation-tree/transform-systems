using Simulation.Tests;
using Transforms.Systems;
using Types;
using Worlds;

namespace Transforms.Tests
{
    public abstract class TransformSystemsTests : SimulationTests
    {
        static TransformSystemsTests()
        {
            MetadataRegistry.Load<TransformsMetadataBank>();
        }

        protected override void SetUp()
        {
            base.SetUp();
            simulator.Add(new TransformSystem());
        }

        protected override void TearDown()
        {
            simulator.Remove<TransformSystem>();
            base.TearDown();
        }

        protected override Schema CreateSchema()
        {
            Schema schema = base.CreateSchema();
            schema.Load<TransformsSchemaBank>();
            return schema;
        }
    }
}