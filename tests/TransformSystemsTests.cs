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
            TypeRegistry.Load<TransformsTypeBank>();
        }

        protected override void SetUp()
        {
            base.SetUp();
            simulator.AddSystem<TransformSystem>();
        }

        protected override Schema CreateSchema()
        {
            Schema schema = base.CreateSchema();
            schema.Load<TransformsSchemaBank>();
            return schema;
        }
    }
}